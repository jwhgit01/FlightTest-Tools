function data = ulg2tt(ulogPath,options)
% Convert ULOG to TimeTable
%
% (C) 2022 Jeremy W. Hopwood <jeremyhopwood@vt.edu>
%
% Description: TODO
%
% Matlab Requirements:  Matlab R2020b or newer
%                       UAV Toolbox
%                       Aerospace Toolbox
%
% The returned timetable may have the following variable names:
%     actuator_controls
%     actuator_outputs
%     esc_rpm
%     estimator_sensor_bias
%     gyro_bias
%     accel_bias
%     mag_bias
%     estimator_status
%     estimator_states
%     input_rc
%     rpm 
%     sensor_baro
%     sensor_combined
%     sensor_gps
%     sensor_mag
%     vehicle_acceleration
%     vehicle_air_data
%     vehicle_angular_acceleration
%     vehicle_angular_velocity
%     vehicle_attitude
%     vehicle_control_mode
%     home_position_lat_lon_alt
%     vehicle_local_position 
%     vehicle_magnetometer
%     vehicle_status 
%     vehicle_status_flags
%     yaw_estimator_status
%     airspeed_validated
%     airdata
%     derived_air_data
%
% Inputs:
%   ulogPath  filepath to .ulg file
%   dt        desired timestep (optional) 
% 
% Outputs:
%   timetable with custom properties constaining ulog information
%

%% Initial Processing

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

% find "duplicate" topic names and append instance to name
% num_actuator_outputs = 0;
for ii = 1:height(msg)
    if msg.InstanceID(ii) > 0
        msg.TopicNames(ii) = msg.TopicNames(ii)+"_"+num2str(msg.InstanceID(ii));
    end
end

% index by row name
topicsAvail = msg.TopicNames;
msg.Properties.RowNames = topicsAvail;

% Parse the options object if given.
if nargin > 1
    dt = options.TimeStep;
    MessageSet = options.MessageSet;
    PlotLogical = options.Plot;
    SaveToMAT = options.SaveToMAT;
else
    dt = 0.1;
    MessageSet = 'Standard';
    PlotLogical = false;
    SaveToMAT = false;
end

% Assign an integer to the given MessageSet. Also determine the time step
% based on the given options.
if strcmp(MessageSet,'Debugging')
    DataLevel = 1;
    DebugLevel = 2;
    Time = seconds(t0_ULOG:dt:tf_ULOG).';
elseif strcmp(MessageSet,'Basic')
    DataLevel = 1;
    DebugLevel = 0;
    Time = seconds(t0_ULOG:dt:tf_ULOG).';
elseif strcmp(MessageSet,'Standard')
    DataLevel = 2;
    DebugLevel = 1;
    Time = seconds(t0_ULOG:dt:tf_ULOG).';
elseif strcmp(MessageSet,'Advanced')
    DataLevel = 3;
    DebugLevel = 2;
    Time = seconds(t0_ULOG:dt:tf_ULOG).';
elseif strcmp(MessageSet,'SystemIdentification')
    DataLevel = 3;
    DebugLevel = 0;
    if any(strcmp(topicsAvail,'sensor_combined'))
        t_raw = seconds(msg('sensor_combined',:).TopicMessages{:}.timestamp);
        dt = median(diff(t_raw));
        Time = seconds(t0_ULOG:dt:tf_ULOG).';
    else
        Time = seconds(t0_ULOG:0.01:tf_ULOG).';
    end
else
    warning('Unknown MessageSet option. Assuming the Basic set.');
    DataLevel = 1;
    DebugLevel = 0;
    Time = seconds(t0_ULOG:dt:tf_ULOG).';
end

% Create the timetables
data = timetable(Time);


%% Metadata
%   Source
%   AvailableTopics
%   Parameters
%   SystemInfo
%   FlightTimestamp
%   HomePosition

% ulog filename
[~,filename,ext] = fileparts(ulogPath);
ULOGfilename = [filename ext];

% add important metadata to table
data = addprop(data,'Source','table');
data = addprop(data,'AvailableTopics','table');
data = addprop(data,'Parameters','table');
data = addprop(data,'SystemInfo','table');
data = addprop(data,'FlightTimestamp','table');
data = addprop(data,'HomePosition','table');
data.Properties.CustomProperties.Source = ULOGfilename;
data.Properties.CustomProperties.AvailableTopics = msg.TopicNames;
data.Properties.CustomProperties.Parameters = params;
data.Properties.CustomProperties.SystemInfo = systeminfo;


%% GPS data
% Determine which GPS topic to use. Priority:
%   1) sensor_gps
%   2) vehicle_gps_position
%   3) vehicle_global_position
sensor_gps_avail = any(strcmp(topicsAvail,'sensor_gps'));
vehicle_global_position_avail = any(strcmp(topicsAvail,'vehicle_global_position'));
vehicle_gps_position_avail = any(strcmp(topicsAvail,'vehicle_gps_position'));
if sensor_gps_avail
    sensor_gps = msg('sensor_gps',:).TopicMessages{:}(:,{'time_utc_usec',...
        'lat','lon','alt','alt_ellipsoid','s_variance_m_s','eph','epv',...
        'vel_n_m_s','vel_e_m_s','vel_d_m_s'});
    gps_avail = true;
    utc_avail = true;
elseif vehicle_gps_position_avail
    sensor_gps = msg('vehicle_gps_position',:).TopicMessages{:}(:,{'time_utc_usec',...
        'lat','lon','alt','alt_ellipsoid','s_variance_m_s','eph','epv',...
        'vel_n_m_s','vel_e_m_s','vel_d_m_s'});
    gps_avail = true;
    utc_avail = true;
elseif vehicle_global_position_avail
    sensor_gps = msg('vehicle_global_position',:).TopicMessages{:}(:,{'lat',...
        'lon','alt','alt_ellipsoid','eph','epv'});
    gps_avail = true;
    utc_avail = false;
else
    gps_avail = false;
    utc_avail = false;
end
%
if utc_avail
    time_utc_usec = sensor_gps.time_utc_usec;
    date_time = datetime(time_utc_usec,'ConvertFrom','epochtime','TicksPerSecond',1e6);
    [HR,MI,SE] = hms(date_time);
    zulu_time = duration(HR,MI,SE,'Format','hh:mm:ss.SS');
    data.Properties.CustomProperties.FlightTimestamp = zulu_time(1);
    sensor_gps = addvars(sensor_gps,zulu_time,'Before','time_utc_usec');
    sensor_gps = removevars(sensor_gps,'time_utc_usec');
end
%
if gps_avail

    % Convert integers to appropriate units
    sensor_gps.lat = double(sensor_gps.lat)*1e-7;
    sensor_gps.lon = double(sensor_gps.lon)*1e-7;
    sensor_gps.alt = double(sensor_gps.alt)*1e-3;
    sensor_gps.alt_ellipsoid = double(sensor_gps.alt_ellipsoid)*1e-3;

    % Add to timetable
    sensor_gps = retime(sensor_gps,Time,'pchip');
    if DataLevel > 2 && utc_avail
        gps_vel_m_s = double([sensor_gps.vel_n_m_s,sensor_gps.vel_e_m_s,sensor_gps.vel_d_m_s]);
        data = addvars(data,sensor_gps.zulu_time,sensor_gps.lat,...
            sensor_gps.lon,sensor_gps.alt,sensor_gps.alt_ellipsoid,...
            double(sensor_gps.s_variance_m_s),double(sensor_gps.eph),...
            double(sensor_gps.epv),gps_vel_m_s,'NewVariableNames',...
            {'zulu_time','lat_deg','lon_deg','alt_m','alt_ellipsoid_m',...
            'gps_vel_variance_m2_s2','eph','epv','gps_vel_m_s'});
    else
        data = addvars(data,sensor_gps.lat,sensor_gps.lon,sensor_gps.alt,...
            'NewVariableNames',{'lat_deg','lon_deg','alt_m'});
    end

    clear sensor_gps time_utc_usec date_time HR MI SE zulu_time gps_vel_m_s

end


%% Level 1 Topics
%   vehicle_local_position
%   vehicle_attitude
%   derived_air_data
%   vehicle_angular_velocity
%   airspeed_validated
%   actuator_controls

% vehicle_local_position
if any(strcmp(topicsAvail,'vehicle_local_position'))
    vehicle_local_position = msg('vehicle_local_position',:).TopicMessages{:};
    if gps_avail
        home_position_alt_lat_lon = [vehicle_local_position.ref_alt(1,1),...
                                     vehicle_local_position.ref_lat(1,1),...
                                     vehicle_local_position.ref_lon(1,1)];
        data.Properties.CustomProperties.HomePosition = home_position_alt_lat_lon;
    end
    vehicle_local_position = vehicle_local_position(:,{'x','y','z',...
        'vx','vy','vz','ax','ay','az','heading'});
    vehicle_local_position = retime(vehicle_local_position,Time,'pchip');
    NED_m = double([vehicle_local_position.x,vehicle_local_position.y,vehicle_local_position.z]);
    vi_m_s = double([vehicle_local_position.vx,vehicle_local_position.vy,vehicle_local_position.vz]);
    ai_m_s = double([vehicle_local_position.ax,vehicle_local_position.ay,vehicle_local_position.az]);
    heading_rad = double(vehicle_local_position.heading);
    data = addvars(data,NED_m,vi_m_s,ai_m_s,heading_rad);
    clear NED_m vi_m_s ai_m_s heading_rad
end 

% vehicle_attitude
if any(strcmp(topicsAvail,'vehicle_attitude'))
    vehicle_attitude = msg('vehicle_attitude',:).TopicMessages{:}(:,'q');
    vehicle_attitude = retime(vehicle_attitude,Time,'pchip');
    R_BI = quat2dcm(vehicle_attitude.q); % rotation from NED to body frame
    [Yaw_rad,Pitch_rad,Roll_rad] = dcm2angle(R_BI,'ZYX'); % 3-2-1 euler parameterization
    EulerAngles_rad = [Roll_rad,Pitch_rad,Yaw_rad]; % euler angles
    data = addvars(data,EulerAngles_rad,permute(R_BI,[3,1,2]),...
        'NewVariableNames',{'EulerAngles_rad','R_BI'});
    clear Yaw_rad Pitch_rad Roll_rad EulerAngles_rad
end

% derived_air_data
if any(strcmp(topicsAvail,'vehicle_attitude')) && any(strcmp(topicsAvail,'vehicle_local_position'))
    timestamp = vehicle_attitude.timestamp;
    temp = retime(vehicle_local_position,timestamp,'pchip');
    vi_m_s = double([temp.vx, temp.vy, temp.vz]);
    N = size(R_BI,3);
    vb_m_s = zeros(N,3);
    alpha_deg_derived = zeros(N,1);
    beta_deg_derived = zeros(N,1);
    V_m_s_derived = zeros(N,1);
    for k = 1:N % loop through timetable
        vb_m_s(k,:) = ((R_BI(:,:,k))*(vi_m_s(k,:).')).'; % body velocity
        V_m_s_derived(k,1) = norm(vi_m_s(k,:)); % derived airspeed (m/s)
        alpha_deg_derived(k,1) = atan2d(vb_m_s(k,3),vb_m_s(k,1)); % angle of attack (deg)
        beta_deg_derived(k,1) = asind(vb_m_s(k,2)/V_m_s_derived(k,1)); % sideslip (deg)
    end
    data = addvars(data,vb_m_s,V_m_s_derived,alpha_deg_derived,beta_deg_derived);
    clear vehicle_attitude vehicle_local_position R_BI
end

% vehicle_angular_velocity
if any(strcmp(topicsAvail,'vehicle_angular_velocity'))
    vehicle_angular_velocity = msg('vehicle_angular_velocity',:).TopicMessages{:};
    vehicle_angular_velocity = retime(vehicle_angular_velocity,Time,'pchip');
    data = addvars(data,vehicle_angular_velocity.xyz,'NewVariableNames','omega_rad_s');
    clear vehicle_angular_velocity
end

% airspeed_validated
if any(strcmp(topicsAvail,'airspeed_validated'))
    airspeed_validated = msg('airspeed_validated',:).TopicMessages{:};
    airspeed_validated(airspeed_validated.airspeed_sensor_measurement_valid==0,:) = [];
    airspeed_validated = airspeed_validated(:,'true_airspeed_m_s');
    airspeed_validated = retime(airspeed_validated,Time,'pchip');
    data = addvars(data,airspeed_validated.true_airspeed_m_s,...
        'NewVariableNames','true_airspeed_m_s');
end

% actuator_controls
if any(strcmp(topicsAvail,'actuator_controls_0'))
    actuator_controls = msg('actuator_controls_0',:).TopicMessages{:}(:,'control');
    actuator_controls = retime(actuator_controls,Time,'pchip');
    data = addvars(data,actuator_controls.control,'NewVariableNames','actuator_controls');
    clear actuator_controls
end


%% Level 2 Topics
%   actuator_outputs
%   input_rc
%   esc_status
%   vehicle_air_data
%   vehicle_angular_acceleration

if DataLevel > 1

    % actuator_outputs
    if any(strcmp(topicsAvail,'actuator_outputs'))
        actuator_outputs = msg('actuator_outputs',:).TopicMessages{:}(:,'output');
        actuator_outputs = retime(actuator_outputs,Time,'nearest');
        data = addvars(data,actuator_outputs.output,'NewVariableNames','actuator_outputs');
        clear actuator_outputs
    end
       
    % input_rc
    if any(strcmp(topicsAvail,'input_rc'))
        input_rc = msg('input_rc',:).TopicMessages{:}(:,'values');
        input_rc = retime(input_rc,Time,'nearest');
        data = addvars(data,input_rc.values,'NewVariableNames','input_rc');
        clear input_rc
    end

    % esc_status
    if any(strcmp(topicsAvail,'esc_status'))
        esc_status = msg('esc_status',:).TopicMessages{:};
        timestamp = esc_status.timestamp;
            
        % get number of ESCs
        esc_count = esc_status.esc_count(1,1);
        
        % loop through number of ESCs
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

        % Add to timetable
        esc_rpm = timetable(timestamp,esc_rpm);
        esc_rpm = retime(esc_rpm,Time,'pchip');
        data = addvars(data,esc_rpm.esc_rpm,'NewVariableNames','esc_rpm');
        clear esc_count timestamp esc_status ni ti ia tidd nidd TT_temp esc_rpm
        
    end

    % vehicle_air_data
    if any(strcmp(topicsAvail,'vehicle_air_data'))
        vehicle_air_data = msg('vehicle_air_data',:).TopicMessages{:}(:,...
            {'baro_alt_meter','baro_temp_celcius','baro_pressure_pa','rho'});
        vehicle_air_data = retime(vehicle_air_data,Time,'pchip');
        data = addvars(data,double(vehicle_air_data.baro_alt_meter),...
            double(vehicle_air_data.baro_temp_celcius),...
            double(vehicle_air_data.baro_pressure_pa),...
            double(vehicle_air_data.rho),...
            'NewVariableNames',{'baro_alt_m','T_C','p_Pa','rho_kg_m3'});
        clear vehicle_air_data
    end

    % vehicle_angular_acceleration
    if any(strcmp(topicsAvail,'vehicle_angular_acceleration'))
        vehicle_angular_acceleration = msg('vehicle_angular_acceleration',:).TopicMessages{:};
        vehicle_angular_acceleration = retime(vehicle_angular_acceleration,Time,'pchip');
        data = addvars(data,vehicle_angular_acceleration.xyz,'NewVariableNames','omegadot_rad_s2');
        clear vehicle_angular_acceleration
    end


end


%% Level 3 Topics
%   rpm
%   sensor_combined
%   sensor_mag
%   yaw_estimator_status
%   estimator_status
%   estimator_states
%   airdata
%   estimator_sensor_bias
%   

if DataLevel > 2

    % rpm
    if any(strcmp(topicsAvail,'rpm'))
        rpm = msg('rpm',:).TopicMessages{:}(:,'indicated_frequency_rpm');
        rpm = retime(rpm,Time,'pchip');
        data = addvars(data,rpm.indicated_frequency_rpm,'NewVariableNames','indicated_rpm');
        clear rpm
    end
    
    % sensor_combined
    if any(strcmp(topicsAvail,'sensor_combined'))
        sensor_combined = msg('sensor_combined',:).TopicMessages{:}(:,...
            {'gyro_rad','accelerometer_m_s2'});
        sensor_combined = retime(sensor_combined,Time,'linear');
        data = addvars(data,sensor_combined.gyro_rad,sensor_combined.accelerometer_m_s2,...
            'NewVariableNames',{'gyro_rad_s','accelerometer_m_s2'});
        clear sensor_combined retime_sensors_combined
    end

    % sensor_mag
    if any(strcmp(topicsAvail,'vehicle_magnetometer'))
        sensor_mag = msg('vehicle_magnetometer',:).TopicMessages{:}(:,'magnetometer_ga');
        sensor_mag = retime(sensor_mag,Time,'pchip');
        data = addvars(data,sensor_mag.magnetometer_ga,'NewVariableNames','magnetometer_ga');
        clear sensor_mag
    elseif any(strcmp(topicsAvail,'sensor_mag'))
        sensor_mag = msg('sensor_mag',:).TopicMessages{:}(:,{'x','y','z'});
        sensor_mag = retime(sensor_mag,Time,'pchip');
        data = addvars(data,[sensor_mag.x,sensor_mag.y,sensor_mag.z],...
            'NewVariableNames','magnetometer_ga');
        clear sensor_mag
    end

    % yaw_estimator_status
    if any(strcmp(topicsAvail,'yaw_estimator_status'))
        yaw_estimator_status = msg('yaw_estimator_status',:).TopicMessages{:}(:,'yaw_variance');
        yaw_estimator_status = retime(yaw_estimator_status,Time,'pchip');
        data = addvars(data,yaw_estimator_status.yaw_variance,...
            'NewVariableNames','yaw_variance_rad2');
        clear yaw_estimator_status
    end

    % estimator_status and/or estimator_states
    % In some cases, the ECL EKF output predictor can diverge (in a spin)
    % Estimator states:
    %   [1:4]   Quaternions
    %   [5:7]   Velocity NED (m/s)
    %   [8:10]  Position NED (m)
    %   [11:13] IMU delta angle bias XYZ (rad)
    %   [14:16] IMU delta velocity bias XYZ (m/s)
    %   [17:19] Earth magnetic field NED (gauss)
    %   [20:22] Body magnetic field XYZ (gauss)
    %   [23:24] Wind velocity NE (m/s)
    % Use of estimator_status and estimator_states changed with v1.12
    if any(strcmp(topicsAvail,'estimator_states'))
        
        % States and covariances
        estimator_states = msg('estimator_states',:).TopicMessages{:}(:,...
            {'states','covariances'});
        
        % Status (add more as needed)
        estimator_status = msg('estimator_status',:).TopicMessages{:}(:,...
            {'pos_horiz_accuracy','pos_vert_accuracy'});
    
        % add to timetable
        estimator_states = retime(estimator_states,Time,'pchip');
        estimator_status = retime(estimator_status,Time,'pchip');
        data = addvars(data,estimator_states.states,estimator_states.covariances,...
            estimator_status.pos_horiz_accuracy,estimator_status.pos_vert_accuracy,...
            'NewVariableNames',{'estimator_states','estimator_covariances',...
            'pos_horiz_accuracy','pos_vert_accuracy'});
        clear estimator_states estimator_status
    
    elseif any(strcmp(topicsAvail,'estimator_status')) % v1.11.3 and earlier
        
        % States, covariances, status
        estimator_states = msg('estimator_status',:).TopicMessages{:}(:,...
            {'states','covariances'});
        estimator_status = msg('estimator_status',:).TopicMessages{:}(:,...
            {'pos_horiz_accuracy','pos_vert_accuracy'});
    
        % add to timetable
        estimator_states = retime(estimator_states,Time,'pchip');
        estimator_status = retime(estimator_status,Time,'pchip');
        data = addvars(data,estimator_states.states,estimator_states.covariances,...
            estimator_status.pos_horiz_accuracy,estimator_status.pos_vert_accuracy,...
            'NewVariableNames',{'estimator_states','estimator_covariances',...
            'pos_horiz_accuracy','pos_vert_accuracy'});
        clear estimator_states estimator_status
    
    end
    
    % airdata
    %   This is a custom message. It has the format:
    %
    %       TODO
    %
    if any(strcmp(topicsAvail,'airdata'))
        airdata = msg('airdata',:).TopicMessages{:};
%         figure
%         hold on
        
        % Remove outliers
        alpha_raw = double(airdata.alpha_deg);
        beta_raw = double(airdata.beta_deg);
        alpha3 = filloutliers(alpha_raw,'linear','movmedian',6);
        beta3 = filloutliers(beta_raw,'linear','movmedian',6);
%         plot(airdata.timestamp,alpha_raw)
%         plot(airdata.timestamp,alpha3)
       
        % Remove noise
        dt = mean(diff(seconds(airdata.timestamp))); % mean time step
        fc = 5; % Hz
        Wn = 2*dt*fc; % fraction of the Nyquist rate used for a digital filter
        [num,den] = butter(5,Wn); % 5th order lowpass filter
        alpha_filt = filtfilt(num,den,alpha3);
        beta_filt = filtfilt(num,den,beta3);

        % Add to timetable
        airdata = addvars(airdata,alpha_filt,beta_filt,'NewVariableNames',...
            {'alpha_filt','beta_filt'});
        airdata = retime(airdata(:,{'alpha_deg','beta_deg','alpha_filt',...
            'beta_filt'}),Time,'pchip','EndValues',0);
        data = addvars(data,airdata.alpha_deg,airdata.beta_deg,...
            airdata.alpha_filt,airdata.beta_filt,'NewVariableNames',...
            {'alpha_vane_deg','beta_vane_deg','alpha_vane_filtered','beta_vane_filtered'});
        clear airdata     
%         plot(Time,airdata.alpha_filt)
%         hold off
%         legend('raw','out','filt')
%         grid on

    end

    % estimator_sensor_bias (gyro_bias, accel_bias, mag_bias)
    if any(strcmp(topicsAvail,'estimator_sensor_bias'))
    
        % Get each sensor's bias estimates
        estimator_sensor_bias = msg('estimator_sensor_bias',:).TopicMessages{:};
        gyro_bias = estimator_sensor_bias(:,'gyro_bias');
        accel_bias = estimator_sensor_bias(:,'accel_bias');
        mag_bias = estimator_sensor_bias(:,'mag_bias');
    
        % Remove invalid estimates
        if any(strcmp(estimator_sensor_bias.Properties.VariableNames,'gyro_bias_valid'))
            gyro_bias(estimator_sensor_bias.gyro_bias_valid==0,:) = [];
            accel_bias(estimator_sensor_bias.accel_bias_valid==0,:) = [];
            mag_bias(estimator_sensor_bias.mag_bias_valid==0,:) = [];
        end
    
        % add to timetable
        gyro_bias = retime(gyro_bias,Time,'pchip');
        accel_bias = retime(accel_bias,Time,'pchip');
        mag_bias = retime(mag_bias,Time,'pchip');
        data = addvars(data,gyro_bias.gyro_bias,accel_bias.accel_bias,...
            mag_bias.mag_bias,'NewVariableNames',...
            {'gyro_bias','accel_bias','mag_bias'});
        clear estimator_sensor_bias gyro_bias accel_bias mag_bias
        
    end


end


%% Debugging Topics
% vehicle_control_mode
% vehicle_status
% vehicle_status_flags

if DebugLevel > 0
    
    % vehicle_control_mode
    if any(strcmp(topicsAvail,'vehicle_control_mode'))
        vehicle_control_mode = msg('vehicle_control_mode',:).TopicMessages{:};
        vehicle_control_mode = retime(vehicle_control_mode,Time,'nearest');
        data = addvars(data,vehicle_control_mode);
        clear vehicle_control_mode
    end

end

if DebugLevel > 1
    
    % vehicle_status
    if any(strcmp(topicsAvail,'vehicle_status'))
        vehicle_status = msg('vehicle_status',:).TopicMessages{:};
        vehicle_status = retime(vehicle_status,Time,'nearest');
        data = addvars(data,vehicle_status);
        clear vehicle_status
    end

    % vehicle_status_flags
    if any(strcmp(topicsAvail,'vehicle_status_flags'))
        vehicle_status_flags = msg('vehicle_status_flags',:).TopicMessages{:};
        vehicle_status_flags = retime(vehicle_status_flags,Time,'nearest');
        data = addvars(data,vehicle_status_flags);
        clear vehicle_status_flags
    end

end


%% Plot
if PlotLogical

    figure
    stackedplot(4,1)
    title('Aircraft States')
    %
    nexttile
    plot(data.Time,data.NED_m)
    ylabel('NED Position (m)')
    legend('North','East','Down')
    grid on
    %
    nexttile
    plot(data.Time,data.EulerAngles_rad*180/pi)
    ylabel('Euler Angles (deg)')
    legend('Roll','Pitch','Yaw')
    grid on
    %
    nexttile
    plot(data.Time,data.vb_m_s)
    ylabel('Body Velocity (m/s)')
    legend('u','v','w')
    grid on
    %
    nexttile
    plot(data.Time,data.omega_rad_s*180/pi)
    ylabel('Angular Velocity (deg/s)')
    legend('p','q','r')
    grid on

end

end