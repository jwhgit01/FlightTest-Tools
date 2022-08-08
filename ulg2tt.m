function TT = ulg2tt(ulogPath,dt)
% Convert ULOG to TimeTable
%
% (C) 2022 Jeremy Hopwood <jeremyhopwood@vt.edu>
%
% Description: TODO
%
% Matlab Requirements:  Matlab R2020b or newer
%                       UAV Toolbox
%                       Aerospace Toolbox
%
% PX4 Requirements: Pixhawk that supports the ECL EKF
%                   GPS
%
% Inputs:
%   ulogPath  filepath to .ulg file
%   dt        desired timestep (optional) 
% 
% Outputs:
%   timetable with custom properties constaining ulog information
%
%% Initial Processing

% desired time step
if nargin < 2
    dt = 0.01; %s
end
% dt_status = 0.1; TODO: slower rate for status messages?

% read ulog file
ulog = ulogreader(ulogPath);

% logger start time
t0_ULOG = seconds(ulog.StartTime);
tf_ULOG = seconds(ulog.EndTime);

% get/show available topics
msg = readTopicMsgs(ulog);

% pixhawk system info
systeminfo = readSystemInformation(ulog);

% parameters
params = readParameters(ulog);

% topics we might want
topicList = {'actuator_controls_0',...
             'actuator_outputs',...
             'airdata',...
             'esc_status',...
             'estimator_states',...
             'estimator_status',...
             'input_rc',...
             'airspeed_validated',...
             'rpm',...
             'safety',...
             'sensor_combined',...
             'sensor_gps',...  % TODO
             'vehicle_air_data',...
             'vehicle_angular_acceleration',...
             'vehicle_angular_velocity',...
             'vehicle_attitude',...
             'vehicle_control_mode',...
             'vehicle_global_position',...
             'vehicle_gps_position',...
             'vehicle_local_position',...
             'vehicle_status',...
             'vehicle_status_flags'}; % TODO
         
% intersection of desired topics and available topics
topicsAvail = intersect(topicList,msg.TopicNames);

% read the topic messages and make the table accessible by row name
data = readTopicMsgs(ulog,'TopicNames',topicsAvail);
%                      'InstanceID',num2cell(zeros(size(topicsAvail))));

% find "duplicate" topic names and append instance to name
num_actuator_outputs = 0;
for ii = 1:height(data)
    
    % the most common we are interested in is actuator_outputs.
    % keep track of how many.
    if strcmp(data.TopicNames(ii),"actuator_outputs")
        num_actuator_outputs = num_actuator_outputs + 1;
    end

    % is the instance id greater than 0
    if data.InstanceID(ii) > 0
        data.TopicNames(ii) = data.TopicNames(ii)+"_"+num2str(data.InstanceID(ii));
    end  

end

% index by row name
data.Properties.RowNames = data.TopicNames;

%% vehicle_gps_position and time synchronization

% if available, do all of the data re-timing based on GPS time.
gpsLogical = any(strcmp(topicsAvail,'vehicle_gps_position'));
if gpsLogical
    
    % get the first gps reading to get timestamps
    vehicle_gps_position = data('vehicle_gps_position',:).TopicMessages{:};
    t0_GPS = seconds(vehicle_gps_position.timestamp(1));
    FlightTimeZulu = datetime(vehicle_gps_position.time_utc_usec(1),...
        'ConvertFrom','epochtime','TicksPerSecond',1e6);

    % determine t0 for uniform timestamps based on first GPS reading
    t0u_GPS = floor(t0_GPS*1/dt)*dt;
    [HR,MI,SE] = hms(datetime(vehicle_gps_position.time_utc_usec(1:2),...
        'ConvertFrom','epochtime','TicksPerSecond',1e6));
    t0_Zulu = duration(HR(1),MI(1),interp1(seconds(vehicle_gps_position.timestamp(1:2)),...
                       SE,t0u_GPS,'linear','extrap'),'Format','hh:mm:ss.SS');

    % uniform time array
    Tu = floor(tf_ULOG*(1/dt))*dt;
    Time = seconds(t0u_GPS:dt:Tu).';
    
    % GPS timestamps
    ZuluTime = (t0_Zulu:seconds(dt):(t0_Zulu+seconds(Tu-t0u_GPS))).'; % uniform GPS timestamps
    
    % create timetable
    TT = timetable(Time,ZuluTime);

else

    % just use the rounded start time of the logger
    t0u = floor(t0_ULOG*(1/dt))*dt;
    Tu = floor(tf_ULOG*(1/dt))*dt;
    Time = seconds(t0u:dt:Tu).';
    
    % create timetable
    TT = timetable(Time); 

end

% number of samples
N = height(TT);

%% vehicle_local_position
%   NED position
%   NED velocity
%   NED acceleration
%   heading
if any(strcmp(topicsAvail,'vehicle_local_position'))
    vehicle_local_position = data('vehicle_local_position',:).TopicMessages{:};
    if gpsLogical
        home_position_lat_lon_alt = [vehicle_local_position.ref_lat(1,1),...
                                     vehicle_local_position.ref_lon(1,1),...
                                     vehicle_local_position.ref_alt(1,1)];
    end
    vehicle_local_position = vehicle_local_position(:,{'x','y','z','vx','vy','vz','ax','ay','az','heading'});
    vehicle_local_position = retime(vehicle_local_position,Time,'pchip'); % retime
    TT = addvars(TT,vehicle_local_position{:,1:3},...
                    vehicle_local_position{:,4:6},...
                    vehicle_local_position{:,7:9},...
                    vehicle_local_position{:,10}*180/pi,...
                    'NewVariableNames',{'NED_m','vi_m_s','ai_m_s2','heading_deg'});
end

%% vehicle_global_position
%   Lat-Lon-Alt position in degrees and feet
%   esllipsoidal alititude in feet
%   terrain altitude in feet
if gpsLogical
    vehicle_global_position = data('vehicle_global_position',:).TopicMessages{:};
    vehicle_global_position = vehicle_global_position(:,{'lat','lon','alt','alt_ellipsoid','terrain_alt'});
    vehicle_global_position = retime(vehicle_global_position,Time,'pchip'); % retime
    LatLon_deg = [vehicle_global_position.lat, vehicle_global_position.lon];
    Alt_ft = vehicle_global_position.alt/0.3048;
    Alt_Ellipsoid_ft = vehicle_global_position.alt_ellipsoid/0.3048;
    Alt_Terrain_ft = vehicle_global_position.terrain_alt/0.3048;
    TT = addvars(TT,LatLon_deg,Alt_ft,Alt_Ellipsoid_ft,Alt_Terrain_ft); % add to timetable
end

%% vehicle_attitude
%   attitude quaternion
%   rotation from NED to body frame
%   Euler Angles
if any(strcmp(topicsAvail,'vehicle_attitude'))
    vehicle_attitude = data('vehicle_attitude',:).TopicMessages{:}; % get timetable
    vehicle_attitude = retime(vehicle_attitude(:,'q'),Time,'pchip'); % retime
    R_BI = quat2dcm(vehicle_attitude.q); % rotation from NED to body frame
    [Yaw_rad,Pitch_rad,Roll_rad] = dcm2angle(R_BI,'ZYX'); % 3-2-1 euler parameterization
    EulerAngles_deg = [Roll_rad,Pitch_rad,Yaw_rad]*180/pi; % euler angles in degrees
    TT = addvars(TT,EulerAngles_deg,permute(R_BI,[3,1,2]),'NewVariableNames',{'EulerAngles_deg','R_BI'}); % add to timetable
end

%% estimator_status and/or estimator_states
% in rare cases, the ECL EKF output predictor can diverge (in a spin)
% estimator states:
%   [1:4]   Quaternions
%   [5:7]   Velocity NED (m/s)
%   [8:10]  Position NED (m)
%   [11:13] IMU delta angle bias XYZ (rad)
%   [14:16] IMU delta velocity bias XYZ (m/s)
%   [17:19] Earth magnetic field NED (gauss)
%   [20:22] Body magnetic field XYZ (gauss)
%   [23:24] Wind velocity NE (m/s)

% use of estimator_status and estimator_states changed with v1.12
if any(strcmp(topicsAvail,'estimator_states'))
    
    % states and covariances
    estimator_states = data('estimator_states',:).TopicMessages{:}; % get timetable
    estimator_states = retime(estimator_states(:,{'states','covariances'}),Time,'pchip'); % retime
    TT = addvars(TT,estimator_states.states,estimator_states.covariances,'NewVariableNames',{'EstimatorStates','EstimatorCovariance'}); % add to timetable
    
    % status (add more as needed)
    estimator_status = data('estimator_status',:).TopicMessages{:}; % get timetable
    estimator_status = retime(estimator_status(:,{'pos_horiz_accuracy','pos_vert_accuracy'}),Time,'pchip'); % retime
    TT = addvars(TT,estimator_status{:,:},'NewVariableNames',{'pos_horiz_vert_accuracy'}); % add to timetable

else
    
    % TODO: add covariances and status topics
    estimator_status = data('estimator_status',:).TopicMessages{:}; % get timetable
    estimator_status = retime(estimator_status(:,1),Time,'pchip'); % retime
    TT = addvars(TT,estimator_status.states,'NewVariableNames',{'EstimatorStates'}); % add to timetable

end

%% Derived Air Data
% From vehicle_attitude and vehicle_local_position we can compute
%   velocity and acceleration in body frame
%   derived airspeed data (airspeed, alpha, beta)
% Note: derived Velocity assumes no wind
if any(strcmp(topicsAvail,'vehicle_attitude')) && any(strcmp(topicsAvail,'vehicle_local_position'))
    vb_m_s = zeros(N,3);
    ab_m_s2 = zeros(N,3);
    alpha_deg_derived = zeros(N,1);
    beta_deg_derived = zeros(N,1);
    V_m_s_derived = zeros(N,1);
    for k = 1:N % loop through timetable
        vb_m_s(k,:) = (squeeze(TT.R_BI(k,:,:))*(TT.vi_m_s(k,:).')).'; % body velocity
        ab_m_s2(k,:) = (squeeze(TT.R_BI(k,:,:))*(TT.ai_m_s2(k,:).')).'; % body accel
        V_m_s_derived(k) = norm(TT.vi_m_s(k,:)); % derived airspeed (m/s)
        alpha_deg_derived(k) = atan2d(vb_m_s(k,3),vb_m_s(k,1)); % angle of attack (deg)
        beta_deg_derived(k) = asind(vb_m_s(k,2)/V_m_s_derived(k)); % sideslip (deg)
    end
    TT = addvars(TT,vb_m_s,ab_m_s2,V_m_s_derived,alpha_deg_derived,beta_deg_derived); % add to timetable
end

%% vehicle_angular_velocity
if any(strcmp(topicsAvail,'vehicle_angular_velocity'))
    vehicle_angular_velocity = data('vehicle_angular_velocity',:).TopicMessages{:};
    vehicle_angular_velocity = retime(vehicle_angular_velocity(:,'xyz'),Time,'pchip'); % retime
    TT = addvars(TT,vehicle_angular_velocity.xyz*180/pi,'NewVariableNames',{'omega_deg_s'}); % add to timetable
end

%% Unwrapped yaw
if any(strcmp(topicsAvail,'vehicle_attitude'))
    yaw_unwrapped = Yaw_rad;
    wrap_counter = 0;
    for k = 2:length(Yaw_rad) 
        if Yaw_rad(k) - Yaw_rad(k-1) > pi
            wrap_counter = wrap_counter - 1;
        end
        if Yaw_rad(k) - Yaw_rad(k-1) < -pi
            wrap_counter = wrap_counter + 1;
        end
        yaw_unwrapped(k) = Yaw_rad(k) + 2*pi*wrap_counter;
    end
    Yaw_unwrapped_deg = yaw_unwrapped*180/pi; % convert to degrees
    TT = addvars(TT,Yaw_unwrapped_deg); % add to timetable
end

%% sensor_combined
if any(strcmp(topicsAvail,'sensor_combined'))
    sensor_combined = data('sensor_combined',:).TopicMessages{:};
    sensor_combined = retime(sensor_combined(:,[1 4]),Time,'pchip'); % retime
    gyro_rad_s = sensor_combined.gyro_rad;
    accel_m_s2 = sensor_combined.accelerometer_m_s2;
    TT = addvars(TT,accel_m_s2,gyro_rad_s,'NewVariableNames',{'accel_m_s2','gyro_rad_s'}); % add to timetable
end

%% input_rc
if any(strcmp(topicsAvail,'input_rc'))
    input_rc = data('input_rc',:).TopicMessages{:};
    input_rc = retime(input_rc(:,'values'),Time,'nearest'); % retime
    TT = addvars(TT,input_rc.values,'NewVariableNames',{'input_rc_PWM'}); % add to timetable
end

%% actuator_controls
if any(strcmp(topicsAvail,'actuator_controls_0'))
    actuator_controls = data('actuator_controls_0',:).TopicMessages{:};
    actuator_controls = retime(actuator_controls(:,'control'),Time,'nearest'); % retime
    act_controls_normalized = actuator_controls.control;
    TT = addvars(TT,act_controls_normalized,'NewVariableNames',{'actuator_controls'}); % add to timetable
end

%% actuator_outputs
if num_actuator_outputs > 0

    % determine the size of the array for efficiency
    nouts = zeros(1,num_actuator_outputs);
    for ii = 1:num_actuator_outputs
        if ii == 1
            nouts(ii) = size(data('actuator_outputs',:).TopicMessages{:}.output,2);
        else
            nouts(ii) = size(data(['actuator_outputs_' num2str(ii-1)],:).TopicMessages{:}.output,2);
        end
    end
    actuator_outputs_array = zeros(N,sum(nouts));
    % although, consider not storing so many zeros...

    % populate it
    for ii = 1:num_actuator_outputs
        if ii == 1
            actuator_outputs = data('actuator_outputs',:).TopicMessages{:};
            actuator_outputs = retime(actuator_outputs(:,'output'),Time,'nearest'); % retime
            actuator_outputs_array(:,1:nouts(ii)) = actuator_outputs.output;
        else
            actuator_outputs = data(['actuator_outputs_' num2str(ii-1)],:).TopicMessages{:};
            actuator_outputs = retime(actuator_outputs(:,'output'),Time,'nearest'); % retime
            actuator_outputs_array(:,sum(nouts(1:ii-1))+1:sum(nouts(1:ii))) = actuator_outputs.output;
        end
    end

    % general aircraft
    TT = addvars(TT,actuator_outputs_array,'NewVariableNames','actuator_outputs_RAW'); % add to timetable
    % units depend on the output (i.e. SERVO, UAVCAN ESC, DSHOT ESC, etc.)
    
    % fixed wing
    % TODO: How do we know which?
    % TT = addvars(TT,actuator_outputs.output(:,[1 2 4]),actuator_outputs.output(:,3),'NewVariableNames',{'deltaPWM_us','ThrottlePWM_us'}); % add to timetable
end

%% vehicle_control_modes
if any(strcmp(topicsAvail,'vehicle_control_mode'))
    vehicle_control_mode = data('vehicle_control_mode',:).TopicMessages{:};
    vehicle_control_mode = retime(vehicle_control_mode,Time,'nearest'); % retime
    TT = addvars(TT,vehicle_control_mode,'NewVariableNames',{'vehicle_control_mode'}); % add to timetable
end

%% airspeed_validated
%   equivalent airspeed
%   true airspeed
if any(strcmp(topicsAvail,'airspeed_validated'))
    airspeed_validated = data('airspeed_validated',:).TopicMessages{:};
    for k = 1:height(airspeed_validated) % remove invalid measurements
        if airspeed_validated.airspeed_sensor_measurement_valid(k) == 0
            airspeed_validated{k,:} = nan(size(airspeed_validated{k,:}));
        end
    end
    %
    % TODO: change from number idx to name
    %
    airspeed_validated = retime(airspeed_validated(:,1:5),Time,'pchip'); % retime 
    EAS_m_s = double(airspeed_validated.equivalent_airspeed_m_s);
    TAS_m_s = double(airspeed_validated.true_airspeed_m_s);
    TT = addvars(TT,EAS_m_s,TAS_m_s,'NewVariableNames',{'EquivalentAirSpeed_m_s','TrueAirSpeed_m_s'}); % add to timetable
end
     
%% airdata
%   measured alpha, beta, V from an air data unit
% Note: this is a custom topic that must be defined in PX4
if any(strcmp(topicsAvail,'airdata'))
    airdata = data('airdata',:).TopicMessages{:};
    %
    % remove noise from alpha and beta vanes before re-timing
    %
    k = 5; % number of neighboring data points to be used
    fc = 1/(2*k*dt); % cutoff frequency
    Wn = 2*dt*fc; % fraction of the Nyquist rate used for a digital filter
    [num,den] = butter(3,Wn); % 3rd order lowpass filter
    airdata.alpha_deg = filtfilt(num,den,double(airdata.alpha_deg));
    airdata.beta_deg = filtfilt(num,den,double(airdata.beta_deg));
    %
    % TODO: consider filter freq based on raw data sample time
    %
    airdata = retime(airdata(:,1:5),Time,'pchip'); % retime
    airspeed_kias = airdata.airspeed_kts;
    alpha_deg = double(airdata.alpha_deg);
    beta_deg = double(airdata.beta_deg);
    TT = addvars(TT,airspeed_kias,alpha_deg,beta_deg,'NewVariableNames',{'airspeed_kias','alpha_deg_measured','beta_deg_measured'}); % add to timetable
end

%% rpm
if any(strcmp(topicsAvail,'rpm'))
    rpm = data('rpm',:).TopicMessages{:};
    rpm = retime(rpm(:,1),Time,'pchip'); % retime
    TT = addvars(TT,double(rpm.indicated_frequency_rpm),'NewVariableNames',{'rpm_indicated'}); % add to timetable
end

%% esc_status
if any(strcmp(topicsAvail,'esc_status'))
    
    esc_status = data('esc_status',:).TopicMessages{:};
    
    % TODO:see how this works for non-UAVCAN ESCs
    % if esc_type = rpm_meas
        
        % get number of ESCs
        esc_count = esc_status.esc_count(1,1);
        
        % loop through number of ESCs
        esc_rpm = zeros(N,esc_count);
        for ii = 1:esc_count
    
            % rpm measurement and timestamp
            ti = double(esc_status.(['esc[' num2str(ii-1) '].timestamp']))/1e6;
            ni = double(esc_status.(['esc[' num2str(ii-1) '].esc_rpm']));
    
            % remove leading and trailing rows with zero
            idx0 = find(ni~=0,1,'first');
            idx1 = find(ni~=0,1,'last');
            ti = ti(idx0:idx1);
            ni = ni(idx0:idx1);
    
            % temporary timetable
            TT_temp = retime(timetable(seconds(ti),ni),Time,'pchip','EndValues',0);
            esc_rpm(:,ii) = TT_temp.ni;
        end
    
        % add to timetable
        TT = addvars(TT,esc_rpm,'NewVariableNames',{'esc_rpm'}); % add to timetable

    % end
end

%% vehicle_status
if any(strcmp(topicsAvail,'vehicle_status'))
    vehicle_status = data('vehicle_status',:).TopicMessages{:};
    vehicle_status = retime(vehicle_status,Time,'nearest'); % retime
    TT = addvars(TT,vehicle_status,'NewVariableNames',{'vehicle_status'}); % add to timetable
end

%% vehicle_status_flags
if any(strcmp(topicsAvail,'vehicle_status_flags'))
    vehicle_status_flags = data('vehicle_status_flags',:).TopicMessages{:};
    vehicle_status_flags = retime(vehicle_status_flags,Time,'nearest'); % retime
    TT = addvars(TT,vehicle_status_flags,'NewVariableNames',{'vehicle_status_flags'}); % add to timetable
end

%% Metadata and Save to mat File

% ulog filename
[~,filename,ext] = fileparts(ulogPath);
ULOGfilename = [filename ext];

% add important metadata to table
TT = addprop(TT,'Source','table');
TT = addprop(TT,'AvailableTopics','table');
TT = addprop(TT,'Parameters','table');
TT = addprop(TT,'SystemInfo','table');
TT = addprop(TT,'FlightTimestamp','table');
TT = addprop(TT,'HomePosition','table');
TT.Properties.CustomProperties.Source = ULOGfilename;
TT.Properties.CustomProperties.AvailableTopics = msg;
TT.Properties.CustomProperties.Parameters = params;
TT.Properties.CustomProperties.SystemInfo = systeminfo;
if gpsLogical
    TT.Properties.CustomProperties.FlightTimestamp = FlightTimeZulu;
    TT.Properties.CustomProperties.HomePosition = home_position_lat_lon_alt;
else


end