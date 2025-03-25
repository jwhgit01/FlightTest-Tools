classdef FlightData < handle

properties
    Data
    UlogFile
    TimeStep(1,1) double {mustBeReal, mustBeFinite, mustBePositive} = 0.1
    Estimation(1,1) logical = false
    CutoffFrequency
end % public properties

properties(SetAccess=private)
    PX4Version
    AvailableMessages
    Parameters
    SystemInfo
    HomePosition
    FlightTimestamp
end % public properties

properties(Access = private)
    % Set upon calling ulg2tt method
    GPSAvailable
    TopicInstances
end % private properties

methods

    function obj = FlightData(UlogFile)
        %FlightData Default constructor

        % If a ulog file is given, add it.
        if nargin > 0
            obj.UlogFile = UlogFile;
        end

    end % FlightData
    
    function ulg2tt(obj,ulogObj)
        %ulg2tt

        % Check input args
        preloadUlog = false;
        if nargin > 1
            mc = metaclass(ulogObj);
            if strcmp(mc.Name,'ulogreader')
                preloadUlog = true;
                ulog = ulogObj;
            else
                error('''ulogObj'' argument must be a ulogreader object.' )
            end
        end

        % Error checking
        if isempty(obj.UlogFile) && ~preloadUlog
            error('Please set the ulog file first or give a ulogreader object.')
        end

        if obj.CutoffFrequency <= 0
            error('Filter cutoff frequency must be positive!')
        end
    
        % read ulog file if not given
        if ~preloadUlog
            ulog = ulogreader(obj.UlogFile);
        end
    
        % data start and stop times
        t_0 = seconds(ulog.StartTime);
        t_f = seconds(ulog.EndTime);

        % Available messages
        obj.AvailableMessages = unique(ulog.AvailableTopics.TopicNames);
        
        % pixhawk system info
        obj.SystemInfo = readSystemInformation(ulog);
        obj.SystemInfo.Properties.RowNames = obj.SystemInfo.InformationField;
        obj.SystemInfo.InformationField = [];
        obj.PX4Version = obj.SystemInfo{"ver_sw_release","Value"};
        
        % parameters
        obj.Parameters = readParameters(ulog);
        obj.Parameters.Properties.RowNames = obj.Parameters.Parameters;
        obj.Parameters.Parameters = [];
    
        % Desired sample times
        dt = obj.TimeStep;
        if obj.Estimation && any(strcmp(obj.AvailableMessages,'sensor_combined'))
            sensor_combined = readTopicMsgs(ulog,"TopicNames","sensor_combined").TopicMessages{1};
            t_raw = seconds(sensor_combined.timestamp);
            dt = median(diff(t_raw));
            obj.TimeStep = dt;
            t_0 = t_raw(1);
            t_f = t_raw(end);
        end
        Time = seconds(t_0:dt:t_f).';
    
        % Create an empty timetable
        obj.Data = timetable(Time);
        
        % Add all available data to the timetable.
        % Note that methods with 'ulog' as the argument generally have multiple
        % instances of interest. Therefore, they are parsed differently.
        %
        obj.GPS(ulog);
        obj.VehicleLocalPosition(ulog);
        obj.VehicleAttitude(ulog);
        obj.BodyVelocity;
        obj.VehicleAngularVelocity(ulog);
        obj.VehicleAngularAcceleration(ulog);
        obj.AirspeedValidated(ulog);
        obj.ActuatorControls(ulog);
        obj.ActuatorMotors(ulog);
        obj.ActuatorServos(ulog);
        obj.ActuatorOutputs(ulog);
        obj.InputRC(ulog);
        obj.ESCStatus(ulog);
        obj.VehicleAirData(ulog);
        obj.RPM(ulog);
        obj.SensorCombined(ulog);
        obj.SensorMag(ulog);
        obj.VehicleMagnetometer(ulog);
        obj.YawEstimatorStatus(ulog);
        obj.EstimatorStates(ulog);
        obj.EstimatorStatus(ulog);
        obj.Airdata(ulog); % Custom

        % TODO:
        % estimator_sensor_bias
        % vehicle_control_mode
        % vehicle_thrust_setpoint
        % vehicle_torque_setpoint
        % vehicle_rates_setpoint
        % vehicle_attitude_setpoint
        % vehicle_local_position_setpoint
        % vehicle_status
        % vehicle_status_flags

        % TODOs:
        %   Remove NaN's after everything is added?
        %   Optional status print outs
        %   Fix ActuatorControls

    end % ulg2tt

    function plot(obj)
        if isempty(obj.Data)
            warning('No data to plot!')
            return
        end
        stackedplot(obj.Data,{'NED_m','EulerAngles_rad','vb_m_s','omega_rad_s','zulu_time'})
    end % plot
    
end % public methods

methods(Access = private)

    function GPS(obj,ulog)
        %GPS

        % Determine which GPS topic to use. Priority:
        %   1) sensor_gps
        %   2) vehicle_gps_position
        %   3) vehicle_global_position
        sensor_gps_avail = any(strcmp(obj.AvailableMessages,'sensor_gps'));
        vehicle_global_position_avail = any(strcmp(obj.AvailableMessages,'vehicle_global_position'));
        vehicle_gps_position_avail = any(strcmp(obj.AvailableMessages,'vehicle_gps_position'));
        if sensor_gps_avail
            gps_data = readTopicMsgs(ulog,"TopicNames","sensor_gps").TopicMessages{1}(:,{...
                'time_utc_usec','lat','lon','alt','alt_ellipsoid','s_variance_m_s',...
                'eph','epv','vel_n_m_s','vel_e_m_s','vel_d_m_s'});
            utc_avail = true;
        elseif vehicle_gps_position_avail
            gps_data = readTopicMsgs(ulog,"TopicNames","vehicle_gps_position").TopicMessages{1}(:,{...
                'time_utc_usec','lat','lon','alt','alt_ellipsoid','s_variance_m_s',...
                'eph','epv','vel_n_m_s','vel_e_m_s','vel_d_m_s'});
            utc_avail = true;
        elseif vehicle_global_position_avail
            gps_data = readTopicMsgs(ulog,"TopicNames","vehicle_global_position").TopicMessages{1}(:,{...
                'lat','lon','alt','alt_ellipsoid','eph','epv'});
            utc_avail = false;
        else
            % No GPS. Nothing to do.
            return
        end
        
        % If GPS time is available, add it
        obj.GPSAvailable = true;
        if utc_avail
            time_utc_usec = gps_data.time_utc_usec;
            date_time = datetime(time_utc_usec,'ConvertFrom','epochtime','TicksPerSecond',1e6);
            [HR,MI,SE] = hms(date_time);
            zulu_time = duration(HR,MI,SE,'Format','hh:mm:ss.SS');
            obj.FlightTimestamp = zulu_time(1);
            gps_data = addvars(gps_data,zulu_time,'Before','time_utc_usec');
            gps_data = removevars(gps_data,'time_utc_usec');
        end
        
        % Convert integers to appropriate units
        gps_data.lat = double(gps_data.lat)*1e-7;
        gps_data.lon = double(gps_data.lon)*1e-7;
        gps_data.alt = double(gps_data.alt)*1e-3;
        gps_data.alt_ellipsoid = double(gps_data.alt_ellipsoid)*1e-3;

        % Filter data if applicable
        fs = 1/median(diff(seconds(gps_data.timestamp)));
        if ~isempty(obj.CutoffFrequency) && (floor(fs) > 2*obj.CutoffFrequency)
            Wn = 2*obj.CutoffFrequency/fs; % fraction of the Nyquist rate
            [num,den] = butter(5,Wn); % 5th order lowpass filter
            for ii = 2:size(gps_data,2)
                gps_data{:,ii} = filtfilt(num,den,gps_data{:,ii});
            end
        end
    
        % Add to timetable
        gps_data = retime(gps_data,obj.Data.Time,'pchip','EndValues',NaN);
        if utc_avail
            gps_vel_m_s = double([gps_data.vel_n_m_s,gps_data.vel_e_m_s,gps_data.vel_d_m_s]);
            obj.Data = addvars(obj.Data,gps_data.zulu_time,gps_data.lat,...
                gps_data.lon,gps_data.alt,gps_data.alt_ellipsoid,...
                double(gps_data.s_variance_m_s),double(gps_data.eph),...
                double(gps_data.epv),gps_vel_m_s,'NewVariableNames',...
                {'zulu_time','lat_deg','lon_deg','alt_m','alt_ellipsoid_m',...
                'gps_vel_variance_m2_s2','eph','epv','gps_vel_m_s'});
        else
            obj.Data = addvars(obj.Data,gps_data.lat,gps_data.lon,gps_data.alt,...
                'NewVariableNames',{'lat_deg','lon_deg','alt_m'});
        end

    end % GPS

    function VehicleLocalPosition(obj,ulog)

        if ~any(strcmp(obj.AvailableMessages,'vehicle_local_position'))
            return
        end

        % Read topic messages
        vehicle_local_position = readTopicMsgs(ulog,"TopicNames","vehicle_local_position").TopicMessages{1};

        % Home position if GPS is availabale
        if obj.GPSAvailable
            home_position_alt_lat_lon = [vehicle_local_position.ref_alt(1,1),...
                                         vehicle_local_position.ref_lat(1,1),...
                                         vehicle_local_position.ref_lon(1,1)];
            obj.HomePosition = home_position_alt_lat_lon;
        end

        % NED frame position, velocity, acceleration, and heading
        vehicle_local_position = vehicle_local_position(:,{'x','y','z',...
            'vx','vy','vz','ax','ay','az','heading'});

        % Filter data if applicable
        fs = 1/median(diff(seconds(vehicle_local_position.timestamp)));
        if ~isempty(obj.CutoffFrequency) && (floor(fs) > 2*obj.CutoffFrequency)
            Wn = 2*obj.CutoffFrequency/fs; % fraction of the Nyquist rate
            [num,den] = butter(5,Wn); % 5th order lowpass filter
            for ii = 1:size(vehicle_local_position,2)
                vehicle_local_position{:,ii} = filtfilt(num,den,...
                    vehicle_local_position{:,ii});
            end
        end

        % Vectroize data and add it to the timetable
        vehicle_local_position = retime(vehicle_local_position,obj.Data.Time,...
            'pchip','EndValues',NaN);
        NED_m = double(vehicle_local_position{:,["x","y","z"]});
        vi_m_s = double(vehicle_local_position{:,["vx","vy","vz"]});
        ai_m_s2 = double(vehicle_local_position{:,["ax","ay","az"]});
        gps_heading_rad = double(vehicle_local_position.heading);
        obj.Data = addvars(obj.Data,NED_m,vi_m_s,ai_m_s2,gps_heading_rad,...
            'NewVariableNames',{'NED_m','vi_m_s','ai_m_s2','gps_heading_rad'});

    end % VehicleLocalPosition

    function VehicleAttitude(obj,ulog)

        if ~any(strcmp(obj.AvailableMessages,'vehicle_attitude'))
            return
        end

        % Read topic messages
        vehicle_attitude = readTopicMsgs(ulog,"TopicNames","vehicle_attitude").TopicMessages{1};

        % Create new timetable from sample timestamp and quaternion
        if any(strcmp(vehicle_attitude.Properties.VariableNames,'timestamp_sample'))
            vehicle_attitude = timetable(vehicle_attitude.timestamp_sample,...
                vehicle_attitude.q,'VariableNames',{'q'});
        else
            vehicle_attitude = vehicle_attitude(:,"q");
            vehicle_attitude.Properties.DimensionNames{1} = 'Time';
        end

        % Filter data if applicable
        fs = 1/median(diff(seconds(vehicle_attitude.Time)));
        if ~isempty(obj.CutoffFrequency) && (floor(fs) > 2*obj.CutoffFrequency)
            Wn = 2*obj.CutoffFrequency/fs; % fraction of the Nyquist rate
            [num,den] = butter(5,Wn); % 5th order lowpass filter
            vehicle_attitude.q = filtfilt(num,den,vehicle_attitude.q);
        end

        % Re-time
        vehicle_attitude = retime(vehicle_attitude,obj.Data.Time,'pchip','EndValues',NaN);

        % Compute rotation matrix and Euler angles from quaternion
        Nq = height(vehicle_attitude);
        R_IB = zeros(3,3,Nq);
        for k = 1:Nq
            q = vehicle_attitude.q(k,:).';
            qvec = q(2:4,1);
            q0 = q(1,1);
            R_BI = (q0^2-qvec'*qvec)*eye(3) + 2*(qvec*qvec') - 2*q0*cpem(qvec);
            R_IB(:,:,k) = R_BI';
        end
        [Yaw_rad,Pitch_rad,Roll_rad] = dcm2angle(pagetranspose(R_IB),'ZYX');
        EulerAngles_rad = [Roll_rad,Pitch_rad,unwrap(Yaw_rad)];

        % Add data to timetable
        obj.Data = addvars(obj.Data,EulerAngles_rad,permute(R_IB,[3,1,2]),...
            'NewVariableNames',{'EulerAngles_rad','R_IB'});

    end % VehicleAttitude

    function BodyVelocity(obj)

        if ( ~any(strcmp(obj.AvailableMessages,'vehicle_attitude')) || ...
                ~any(strcmp(obj.AvailableMessages,'vehicle_local_position')) )
            return
        end
        
        % Get data from vehicle_attitude and vehicle_local_position
        N = height(obj.Data);
        vi_m_s = obj.Data.vi_m_s;
        ai_m_s2 = obj.Data.ai_m_s2;
        vb_m_s = zeros(N,3);
        ab_m_s2 = zeros(N,3);
        alpha_deg_derived = zeros(N,1);
        beta_deg_derived = zeros(N,1);
        V_m_s_derived = zeros(N,1);

        % Loop through each sample and compute body frame quantities
        for k = 1:N % loop through timetable
            R_IB_k = squeeze(obj.Data{k,"R_IB"});
            vb_m_s(k,:) = (R_IB_k'*(vi_m_s(k,:).')).'; % body velocity
            ab_m_s2(k,:) = (R_IB_k'*(ai_m_s2(k,:).')).'; % inertial acceleration in body frame
            V_m_s_derived(k,1) = norm(vi_m_s(k,:)); % derived airspeed (m/s)
            alpha_deg_derived(k,1) = atan2d(vb_m_s(k,3),vb_m_s(k,1)); % angle of attack (deg)
            beta_deg_derived(k,1) = asind(vb_m_s(k,2)/V_m_s_derived(k,1)); % sideslip (deg)
        end

        % Add to timetable
        obj.Data = addvars(obj.Data,vb_m_s,ab_m_s2,V_m_s_derived,...
            alpha_deg_derived,beta_deg_derived,'NewVariableNames',...
            {'vb_m_s','ab_m_s2','V_m_s_derived',...
            'alpha_deg_derived','beta_deg_derived'});

    end % BodyVelocity

    function VehicleAngularVelocity(obj,ulog)

        if ~any(strcmp(obj.AvailableMessages,'vehicle_angular_velocity'))
            return
        end

        % Read topic messages
        vehicle_angular_velocity = readTopicMsgs(ulog,"TopicNames","vehicle_angular_velocity").TopicMessages{1};

        % Create new timetable from sample timestamp
        if any(strcmp(vehicle_angular_velocity.Properties.VariableNames,'xyz_derivative'))
            omegadot = true;
            vehicle_angular_velocity = timetable(vehicle_angular_velocity.timestamp_sample,...
                vehicle_angular_velocity.xyz,vehicle_angular_velocity.xyz_derivative,...
                'VariableNames',{'omega_rad_s','omegadot_rad_s2'});
        else
            omegadot = false;
            vehicle_angular_velocity = timetable(vehicle_angular_velocity.timestamp_sample,...
                vehicle_angular_velocity.xyz,'VariableNames',{'omega_rad_s'});
        end
     
        % Filter data if applicable
        fs = 1/median(diff(seconds(vehicle_angular_velocity.Time)));
        if ~isempty(obj.CutoffFrequency) && (floor(fs) > 2*obj.CutoffFrequency)
            Wn = 2*obj.CutoffFrequency/fs; % fraction of the Nyquist rate
            [num,den] = butter(5,Wn); % 5th order lowpass filter
            for ii = 1:size(vehicle_angular_velocity,2)
                vehicle_angular_velocity{:,ii} = filtfilt(num,den,...
                    vehicle_angular_velocity{:,ii});
            end
        end

        % Re-time and add data to timetable
        vehicle_angular_velocity = retime(vehicle_angular_velocity,obj.Data.Time,...
            'pchip','EndValues',NaN);
        obj.Data = addvars(obj.Data,vehicle_angular_velocity.omega_rad_s,...
                'NewVariableNames',{'omega_rad_s'});
        if omegadot
            obj.Data = addvars(obj.Data,vehicle_angular_velocity.omegadot_rad_s2,...
                'NewVariableNames',{'omegadot_rad_s2'});
        end

    end % VehicleAngularVelocity

    function VehicleAngularAcceleration(obj,ulog)

        if ~any(strcmp(obj.AvailableMessages,'vehicle_angular_acceleration'))
            return
        end

        % Skip if we got angular acceleration from VehicleAngularVelocity.
        if any(strcmp(obj.Data.Properties.VariableNames,'omegadot_rad_s2'))
            return
        end

        % Read topic messages
        vehicle_angular_acceleration = readTopicMsgs(ulog,"TopicNames","vehicle_angular_acceleration").TopicMessages{1};

        % Create new timetable from sample timestamp
        vehicle_angular_acceleration = timetable(vehicle_angular_acceleration.timestamp_sample,...
            vehicle_angular_acceleration.xyz,'VariableNames',{'omegadot_rad_s2'});
     
        % Filter data if applicable
        fs = 1/median(diff(seconds(vehicle_angular_acceleration.Time)));
        if ~isempty(obj.CutoffFrequency) && (floor(fs) > 2*obj.CutoffFrequency)
            Wn = 2*obj.CutoffFrequency/fs; % fraction of the Nyquist rate
            [num,den] = butter(5,Wn); % 5th order lowpass filter
            vehicle_angular_acceleration.omegadot_rad_s2 = filtfilt(num,den,...
                    vehicle_angular_acceleration.omegadot_rad_s2);
        end

        % Re-time and add data to timetable
        vehicle_angular_acceleration = retime(vehicle_angular_acceleration,...
            obj.Data.Time,'pchip','EndValues',NaN);
        obj.Data = addvars(obj.Data,vehicle_angular_acceleration.omegadot_rad_s2,...
                'NewVariableNames',{'omegadot_rad_s2'});

    end % VehicleAngularAcceleration

    function AirspeedValidated(obj,ulog)

        if ~any(strcmp(obj.AvailableMessages,'airspeed_validated'))
            return
        end

        % Read topic messages
        airspeed_validated = readTopicMsgs(ulog,"TopicNames","airspeed_validated").TopicMessages{1};

        % Remove invalid measurements
        airspeed_validated(airspeed_validated.airspeed_sensor_measurement_valid==0,:) = [];

        % Keep only indicated and true airspeed
        airspeed_validated = airspeed_validated(:,["indicated_airspeed_m_s",...
            "true_airspeed_m_s"]);
        airspeed_validated.indicated_airspeed_m_s = double(airspeed_validated.indicated_airspeed_m_s);
        airspeed_validated.true_airspeed_m_s = double(airspeed_validated.true_airspeed_m_s);

        % Filter data if applicable
        fs = 1/median(diff(seconds(airspeed_validated.timestamp)));
        if ~isempty(obj.CutoffFrequency) && (floor(fs) > 2*obj.CutoffFrequency)
            Wn = 2*obj.CutoffFrequency/fs; % fraction of the Nyquist rate
            [num,den] = butter(5,Wn); % 5th order lowpass filter
            airspeed_validated.indicated_airspeed_m_s = filtfilt(num,den,...
                airspeed_validated.indicated_airspeed_m_s);
            airspeed_validated.true_airspeed_m_s = filtfilt(num,den,...
                airspeed_validated.true_airspeed_m_s);
        end

        % Re-time and add data to timetable
        airspeed_validated = retime(airspeed_validated,obj.Data.Time,...
            'pchip','EndValues',NaN);
        obj.Data = addvars(obj.Data,airspeed_validated.indicated_airspeed_m_s,...
            airspeed_validated.true_airspeed_m_s,'NewVariableNames',...
            {'indicated_airspeed_m_s','true_airspeed_m_s'});

    end % AirspeedValidated

    function ActuatorControls(obj,ulog)
        % TODO: Fix this to include actuator_control_1 and _2

        if ~any(strcmp(obj.AvailableMessages,'actuator_controls_0'))
            return
        end

        % Read topic messages
        actuator_controls_0 = readTopicMsgs(ulog,"TopicNames","actuator_controls_0").TopicMessages{1};

        % Create new timetable from sample timestamps as long as they are all
        % there (this can happen in older versions of PX4).
        if ~any(seconds(actuator_controls_0.timestamp_sample) == 0)
            actuator_controls_0 = timetable(actuator_controls_0.timestamp_sample,...
                actuator_controls_0.control,'VariableNames',{'control'});
        else
            actuator_controls_0 = actuator_controls_0(:,"control");
            actuator_controls_0.Properties.DimensionNames{1} = 'Time';
        end

        % Filter data if applicable
        fs = 1/median(diff(seconds(actuator_controls_0.Time)));
        if ~isempty(obj.CutoffFrequency) && (floor(fs) > 2*obj.CutoffFrequency)
            Wn = 2*obj.CutoffFrequency/fs; % fraction of the Nyquist rate
            [num,den] = butter(5,Wn); % 5th order lowpass filter
            actuator_controls_0.control = filtfilt(num,den,...
                actuator_controls_0.control);
        end

        % Re-time and add data to timetable
        actuator_controls_0 = retime(actuator_controls_0,obj.Data.Time,...
            'pchip','EndValues',NaN);
        obj.Data = addvars(obj.Data,actuator_controls_0.control,...
            'NewVariableNames',{'actuator_controls'});

    end % ActuatorControls

    function ActuatorMotors(obj,ulog)

        if ~any(strcmp(obj.AvailableMessages,'actuator_motors'))
            return
        end

        % Read topic messages
        actuator_motors = readTopicMsgs(ulog,"TopicNames","actuator_motors").TopicMessages{1};

        % Create new timetable from sample timestamp
        actuator_motors = timetable(actuator_motors.timestamp_sample,...
            actuator_motors.control,'VariableNames',{'control'});

        % Remove columns of NaN
        idx = all(isnan(actuator_motors.control));
        actuator_motors.control(:,idx) = [];

        % Filter data if applicable
        fs = 1/median(diff(seconds(actuator_motors.Time)));
        if ~isempty(obj.CutoffFrequency) && (floor(fs) > 2*obj.CutoffFrequency)
            Wn = 2*obj.CutoffFrequency/fs; % fraction of the Nyquist rate
            [num,den] = butter(5,Wn); % 5th order lowpass filter
            actuator_motors.control = filtfilt(num,den,actuator_motors.control);
        end

        % Re-time and add data to timetable
        actuator_motors = retime(actuator_motors,obj.Data.Time,...
            'pchip','EndValues',NaN);
        obj.Data = addvars(obj.Data,actuator_motors.control,...
            'NewVariableNames',{'actuator_motors'});

    end % ActuatorMotors

    function ActuatorServos(obj,ulog)

        if ~any(strcmp(obj.AvailableMessages,'actuator_servos'))
            return
        end

        % Read topic messages
        actuator_servos = readTopicMsgs(ulog,"TopicNames","actuator_servos").TopicMessages{1};

        % Create new timetable from sample timestamp
        actuator_servos = timetable(actuator_servos.timestamp_sample,...
            actuator_servos.control,'VariableNames',{'control'});

        % Remove columns of NaN
        idx = all(isnan(actuator_servos.control));
        actuator_servos.control(:,idx) = [];

        % Filter data if applicable
        fs = 1/median(diff(seconds(actuator_servos.Time)));
        if ~isempty(obj.CutoffFrequency) && (floor(fs) > 2*obj.CutoffFrequency)
            Wn = 2*obj.CutoffFrequency/fs; % fraction of the Nyquist rate
            [num,den] = butter(5,Wn); % 5th order lowpass filter
            actuator_servos.control = filtfilt(num,den,actuator_servos.control);
        end

        % Re-time and add data to timetable
        actuator_servos = retime(actuator_servos,obj.Data.Time,...
            'pchip','EndValues',NaN);
        obj.Data = addvars(obj.Data,actuator_servos.control,...
            'NewVariableNames',{'actuator_servos'});

    end % ActuatorServos

    function ActuatorOutputs(obj,ulog)

        if ~any(strcmp(obj.AvailableMessages,'actuator_outputs'))
            return
        end

        % Loop through all instances of actuator_outputs
        actuator_outputs_msgs = readTopicMsgs(ulog,"TopicNames","actuator_outputs");
        for ii = 1:height(actuator_outputs_msgs)

            % Get topic messages
            instance = actuator_outputs_msgs.InstanceID(ii);
            actuator_outputs = actuator_outputs_msgs.TopicMessages{ii};

            % Remove extra columns
            noutputs = max(actuator_outputs.noutputs);
            actuator_outputs.output = actuator_outputs.output(:,1:noutputs);
            actuator_outputs = actuator_outputs(:,"output");
    
            % Filter data if applicable
            fs = 1/median(diff(seconds(actuator_outputs.timestamp)));
            if ~isempty(obj.CutoffFrequency) && (floor(fs) > 2*obj.CutoffFrequency)
                Wn = 2*obj.CutoffFrequency/fs; % fraction of the Nyquist rate
                [num,den] = butter(5,Wn); % 5th order lowpass filter
                actuator_outputs.output = filtfilt(num,den,actuator_outputs.output);
            end
    
            % Re-time and add data to timetable
            actuator_outputs = retime(actuator_outputs,obj.Data.Time,...
                'pchip','EndValues',NaN);
            obj.Data = addvars(obj.Data,actuator_outputs.output,...
                'NewVariableNames',{['actuator_outputs_' num2str(instance)]});

        end

    end % ActuatorOutputs

    function InputRC(obj,ulog)

        if ~any(strcmp(obj.AvailableMessages,'input_rc'))
            return
        end

        % Read topic messages
        input_rc = readTopicMsgs(ulog,"TopicNames","input_rc").TopicMessages{1};

        % Re-time and add data to timetable
        input_rc = retime(input_rc,obj.Data.Time,'nearest','EndValues',NaN);
        obj.Data = addvars(obj.Data,input_rc.values,'NewVariableNames',{'input_rc'});

    end % InputRC

    function ESCStatus(obj,ulog)

        if ~any(strcmp(obj.AvailableMessages,'esc_status'))
            return
        end

        % Read topic messages
        esc_status = readTopicMsgs(ulog,"TopicNames","esc_status").TopicMessages{1};
        timestamp = esc_status.timestamp;
                
        % Get number of ESCs
        esc_count = esc_status.esc_count(1,1);
            
        % Loop through ESCs and get RPM data
        esc_rpm = zeros(height(esc_status),esc_count);
        for ii = 1:esc_count
    
            % rpm measurement and timestamp
            ni = double(esc_status.(['esc[' num2str(ii-1) '].esc_rpm']));
            
            % remove duplicate timestamp values and keep first sample
            [ti,ia,~] = unique(esc_status.(['esc[' num2str(ii-1) '].timestamp']),'first');
            tidd = double(ti)/1e6;
            nidd = ni(ia);
    
            % temporary timetable
            TT_temp = retime(timetable(seconds(tidd),nidd),timestamp,'pchip','EndValues',0);
            esc_rpm(:,ii) = TT_temp.nidd;

        end

        % Filter data if applicable
        fs = 1/median(diff(seconds(timestamp)));
        if ~isempty(obj.CutoffFrequency) && (floor(fs) > 2*obj.CutoffFrequency)
            Wn = 2*obj.CutoffFrequency/fs; % fraction of the Nyquist rate
            [num,den] = butter(5,Wn); % 5th order lowpass filter
            esc_rpm = filtfilt(num,den,esc_rpm);
        end

        % Add to timetable
        esc_rpm = timetable(timestamp,esc_rpm);
        esc_rpm = retime(esc_rpm,obj.Data.Time,'pchip','EndValues',NaN);
        obj.Data = addvars(obj.Data,esc_rpm.esc_rpm,'NewVariableNames',{'esc_rpm'});

    end % ESCStatus

    function VehicleAirData(obj,ulog)

        if ~any(strcmp(obj.AvailableMessages,'vehicle_air_data'))
            return
        end

        % Read topic messages
        vehicle_air_data = readTopicMsgs(ulog,"TopicNames","vehicle_air_data").TopicMessages{1};

        % Change timestamps to sample timestamps
        vehicle_air_data.timestamp = vehicle_air_data.timestamp_sample;
        vehicle_air_data = vehicle_air_data(:,["baro_alt_meter",...
            "baro_temp_celcius","baro_pressure_pa","rho"]);

        % Cast to double precision
        vehicle_air_data.baro_alt_meter = double(vehicle_air_data.baro_alt_meter);
        vehicle_air_data.baro_temp_celcius = double(vehicle_air_data.baro_temp_celcius);
        vehicle_air_data.baro_pressure_pa = double(vehicle_air_data.baro_pressure_pa);
        vehicle_air_data.rho = double(vehicle_air_data.rho);

        % Filter data if applicable
        fs = 1/median(diff(seconds(vehicle_air_data.timestamp)));
        if ~isempty(obj.CutoffFrequency) && (floor(fs) > 2*obj.CutoffFrequency)
            Wn = 2*obj.CutoffFrequency/fs; % fraction of the Nyquist rate
            [num,den] = butter(5,Wn); % 5th order lowpass filter
            for ii = 1:size(vehicle_air_data,2)
                vehicle_air_data{:,ii} = filtfilt(num,den,...
                    double(vehicle_air_data{:,ii}));
            end
        end

        % Re-time and add to timetable
        vehicle_air_data = retime(vehicle_air_data,obj.Data.Time,'pchip','EndValues',NaN);
        obj.Data = addvars(obj.Data,vehicle_air_data.baro_alt_meter,...
            vehicle_air_data.baro_temp_celcius,vehicle_air_data.baro_pressure_pa,...
            vehicle_air_data.rho,'NewVariableNames',{'baro_alt_m','baro_temp_c',...
            'baro_pressure_pa','rho_kg_m3'});

    end % VehicleAirData

    function RPM(obj,ulog)

        if ~any(strcmp(obj.AvailableMessages,'rpm'))
            return
        end

        % Read topic messages
        rpm = readTopicMsgs(ulog,"TopicNames","rpm").TopicMessages{1}(:,"indicated_frequency_rpm");
        rpm.indicated_frequency_rpm = double(rpm.indicated_frequency_rpm);

        % Remove outliers due to Thunderfly bug
        rpm = rmoutliers(rpm,"percentiles",[0 99]);

        % Filter data if applicable
        fs = 1/median(diff(seconds(rpm.timestamp)));
        if ~isempty(obj.CutoffFrequency) && (floor(fs) > 2*obj.CutoffFrequency)
            Wn = 2*obj.CutoffFrequency/fs; % fraction of the Nyquist rate
            [num,den] = butter(5,Wn); % 5th order lowpass filter
            rpm.indicated_frequency_rpm = filtfilt(num,den,...
                    rpm.indicated_frequency_rpm);
        end

        % Re-time and add data to timetable
        rpm = retime(rpm,obj.Data.Time,'pchip','EndValues',NaN);
        obj.Data = addvars(obj.Data,rpm.indicated_frequency_rpm,...
                'NewVariableNames',{'indicated_rpm'});

    end % RPM

    function SensorCombined(obj,ulog)

        if ~any(strcmp(obj.AvailableMessages,'sensor_combined'))
            return
        end

        % Read topic messages
        sensor_combined = readTopicMsgs(ulog,"TopicNames","sensor_combined").TopicMessages{1}(:,["gyro_rad","accelerometer_m_s2"]);
        
        % Filter data if applicable
        fs = 1/median(diff(seconds(sensor_combined.timestamp)));
        if ~isempty(obj.CutoffFrequency) && (floor(fs) > 2*obj.CutoffFrequency)
            Wn = 2*obj.CutoffFrequency/fs; % fraction of the Nyquist rate
            [num,den] = butter(5,Wn); % 5th order lowpass filter
            sensor_combined.gyro_rad = filtfilt(num,den,sensor_combined.gyro_rad);
            sensor_combined.accelerometer_m_s2 = filtfilt(num,den,sensor_combined.accelerometer_m_s2);
        end

        % Re-time and add to timetable
        sensor_combined = retime(sensor_combined,obj.Data.Time,'pchip','EndValues',NaN);
        obj.Data = addvars(obj.Data,sensor_combined.gyro_rad,...
            sensor_combined.accelerometer_m_s2,'NewVariableNames',...
            {'gyro_rad_s','accelerometer_m_s2'});

    end % SensorCombined

    function SensorMag(obj,ulog)

        if ~any(strcmp(obj.AvailableMessages,'sensor_mag'))
            return
        end

        % Loop through all instances of actuator_outputs
        sensor_mag_msgs = readTopicMsgs(ulog,"TopicNames","sensor_mag");
        for ii = 1:height(sensor_mag_msgs)

            % Get topic messages
            instance = sensor_mag_msgs.InstanceID(ii);
            sensor_mag = sensor_mag_msgs.TopicMessages{ii};

            % Create new timetable from sample timestamp
            sensor_mag = timetable(sensor_mag.timestamp_sample,...
                sensor_mag{:,["x","y","z"]},'VariableNames',{'b'});
    
            % Filter data if applicable
            fs = 1/median(diff(seconds(sensor_mag.Time)));
            if ~isempty(obj.CutoffFrequency) && (floor(fs) > 2*obj.CutoffFrequency)
                Wn = 2*obj.CutoffFrequency/fs; % fraction of the Nyquist rate
                [num,den] = butter(5,Wn); % 5th order lowpass filter
                sensor_mag.b = filtfilt(num,den,sensor_mag.b);
            end
    
            % Re-time and add data to timetable
            sensor_mag = retime(sensor_mag,obj.Data.Time,'pchip','EndValues',NaN);
            obj.Data = addvars(obj.Data,sensor_mag.b,...
                'NewVariableNames',{['magnetometer_' num2str(instance) '_ga']});

        end

    end % SensorMag

    function VehicleMagnetometer(obj,ulog)

        if ~any(strcmp(obj.AvailableMessages,'vehicle_magnetometer'))
            return
        end

        % Read topic messages
        vehicle_magnetometer = readTopicMsgs(ulog,"TopicNames","vehicle_magnetometer").TopicMessages{1}(:,"magnetometer_ga");
        
        % Filter data if applicable
        fs = 1/median(diff(seconds(vehicle_magnetometer.timestamp)));
        if ~isempty(obj.CutoffFrequency) && (floor(fs) > 2*obj.CutoffFrequency)
            Wn = 2*obj.CutoffFrequency/fs; % fraction of the Nyquist rate
            [num,den] = butter(5,Wn); % 5th order lowpass filter
            vehicle_magnetometer.magnetometer_ga = filtfilt(num,den,...
                vehicle_magnetometer.magnetometer_ga);
        end

        % Re-time and add to timetable
        vehicle_magnetometer = retime(vehicle_magnetometer,obj.Data.Time,...
            'pchip','EndValues',NaN);
        obj.Data = addvars(obj.Data,vehicle_magnetometer.magnetometer_ga,...
            'NewVariableNames',{'magnetometer_fused_ga'});

    end % VehicleMagnetometer

    function YawEstimatorStatus(obj,ulog)

        if ~any(strcmp(obj.AvailableMessages,'yaw_estimator_status'))
            return
        end

        % Read topic messages
        yaw_estimator_status = readTopicMsgs(ulog,"TopicNames","yaw_estimator_status").TopicMessages{1};

        % Create new timetable from sample timestamps
        if any(strcmp(yaw_estimator_status.Properties.VariableNames,'timestamp_sample'))
            yaw_estimator_status = timetable(yaw_estimator_status.timestamp_sample,...
            yaw_estimator_status.yaw_variance,'VariableNames',{'yaw_variance'});
        else
            yaw_estimator_status = yaw_estimator_status(:,"yaw_variance");
            yaw_estimator_status.Properties.DimensionNames{1} = 'Time';
        end
        
        % Filter data if applicable
        fs = 1/median(diff(seconds(yaw_estimator_status.Time)));
        if ~isempty(obj.CutoffFrequency) && (floor(fs) > 2*obj.CutoffFrequency)
            Wn = 2*obj.CutoffFrequency/fs; % fraction of the Nyquist rate
            [num,den] = butter(5,Wn); % 5th order lowpass filter
            yaw_estimator_status.yaw_variance = filtfilt(num,den,...
                yaw_estimator_status.yaw_variance);
        end

        % Re-time and add to timetable
        yaw_estimator_status = retime(yaw_estimator_status,obj.Data.Time,...
            'pchip','EndValues',NaN);
        obj.Data = addvars(obj.Data,yaw_estimator_status.yaw_variance,...
            'NewVariableNames',{'yaw_variance_rad2'});

    end % YawEstimatorStatus

    function EstimatorStates(obj,ulog)

        if ~any(strcmp(obj.AvailableMessages,'estimator_states'))
            return
        end

        % Read topic messages
        estimator_states = readTopicMsgs(ulog,"TopicNames","estimator_states").TopicMessages{1};

        % Create new timetable from sample timestamps
        estimator_states = timetable(estimator_states.timestamp_sample,...
            estimator_states.states,estimator_states.covariances,...
            'VariableNames',{'estimator_states','estimator_variances'});
        
        % Filter data if applicable
        fs = 1/median(diff(seconds(estimator_states.Time)));
        if ~isempty(obj.CutoffFrequency) && (floor(fs) > 2*obj.CutoffFrequency)
            Wn = 2*obj.CutoffFrequency/fs; % fraction of the Nyquist rate
            [num,den] = butter(5,Wn); % 5th order lowpass filter
            estimator_states.estimator_states = filtfilt(num,den,...
                estimator_states.estimator_states);
            estimator_states.estimator_variances = filtfilt(num,den,...
                estimator_states.estimator_variances);
        end

        % Re-time and add to timetable
        estimator_states = retime(estimator_states,obj.Data.Time,...
            'pchip','EndValues',NaN);
        obj.Data = addvars(obj.Data,estimator_states.estimator_states,...
            estimator_states.estimator_variances,'NewVariableNames',...
            {'estimator_states','estimator_variances'});

    end % EstimatorStates

    function EstimatorStatus(obj,ulog)

        if ~any(strcmp(obj.AvailableMessages,'estimator_status'))
            return
        end

        % Read topic messages
        estimator_status = readTopicMsgs(ulog,"TopicNames","estimator_status").TopicMessages{1};

        % If using PX4 v1.11.3 or earlier, get states and variances
        if any(strcmp(estimator_status.Properties.VariableNames,'states'))
            
            % Get states and covariances
            estimator_states = estimator_status(:,["states","covariances"]);
        
            % Filter data if applicable
            fs = 1/median(diff(seconds(estimator_states.timestamp)));
            if ~isempty(obj.CutoffFrequency) && (floor(fs) > 2*obj.CutoffFrequency)
                Wn = 2*obj.CutoffFrequency/fs; % fraction of the Nyquist rate
                [num,den] = butter(5,Wn); % 5th order lowpass filter
                estimator_states.states = filtfilt(num,den,...
                    estimator_states.states);
                estimator_states.covariances = filtfilt(num,den,...
                    estimator_states.covariances);
            end

            % Re-time and add to timetable
            estimator_states = retime(estimator_states,obj.Data.Time,...
                'pchip','EndValues',NaN);
            obj.Data = addvars(obj.Data,estimator_states.states,...
                estimator_states.covariances,'NewVariableNames',...
                {'estimator_states','estimator_variances'});
        end

        % Get desired estimator staus topics
        estimator_status = estimator_status(:,["pos_horiz_accuracy",...
            "pos_vert_accuracy"]);

        % If is exists, set the times to the sample timestamps
        if any(strcmp(estimator_status.Properties.VariableNames,'timestamp_sample'))
            estimator_status.timestamp = estimator_status.timestamp_sample;
            estimator_status(:,"timestamp_sample") = [];
        end

        % Re-time and add to timetable
        estimator_status = retime(estimator_status,obj.Data.Time,...
            'pchip','EndValues',NaN);
        obj.Data = addvars(obj.Data,estimator_status.pos_horiz_accuracy,...
            estimator_status.pos_vert_accuracy,'NewVariableNames',...
            {'pos_horiz_accuracy','pos_vert_accuracy'});

    end % EstimatorStatus

    function Airdata(obj,ulog)

        if ~any(strcmp(obj.AvailableMessages,'airdata'))
            return
        end

        % Read topic messages
        airdata = readTopicMsgs(ulog,"TopicNames","airdata").TopicMessages{1};

        % The variable names/units have changed over the years...
        if any(strcmp(airdata.Properties.VariableNames,'alpha_deg'))
            alpha_rad = airdata.alpha_deg*pi/180;
            beta_rad = airdata.beta_deg*pi/180;
        elseif any(strcmp(airdata.Properties.VariableNames,'alpha_rad'))
            alpha_rad = airdata.alpha_rad;
            beta_rad = airdata.beta_rad;
        elseif any(strcmp(airdata.Properties.VariableNames,'alpha_vane'))
            alpha_rad = airdata.alpha_vane;
            beta_rad = airdata.beta_vane;
        else
            warning('Custom message ''airdata'' does not have recognized variable names.')
            return
        end
        airdata = timetable(airdata.timestamp,double(alpha_rad),double(beta_rad),...
            'VariableNames',{'alpha_rad','beta_rad'});

        % Filter data if applicable
        fs = 1/median(diff(seconds(airdata.Time)));
        if ~isempty(obj.CutoffFrequency) && (floor(fs) > 2*obj.CutoffFrequency)
            Wn = 2*obj.CutoffFrequency/fs; % fraction of the Nyquist rate
            [num,den] = butter(5,Wn); % 5th order lowpass filter
            airdata.alpha_rad = filtfilt(num,den,airdata.alpha_rad);
            airdata.beta_rad = filtfilt(num,den,airdata.beta_rad);
        end

        % Re-time and add data to timetable
        airdata = retime(airdata,obj.Data.Time,'pchip','EndValues',NaN);
        obj.Data = addvars(obj.Data,airdata.alpha_rad,airdata.beta_rad,...
                'NewVariableNames',{'alpha_vane_rad','beta_vane_rad'});

    end % Airdata

end

end