function TT = ulg2tt(ulogPath)
% Convert ULOG to TimeTable
%
% (C) 2021 Jeremy Hopwood <jeremyhopwood@vt.edu>
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
% 
% Outputs:
%   timetable with custom properties constaining ulog information
%
%% initial processing

% read ulog file
ulog = ulogreader(ulogPath);

% get/show available topics
msg = readTopicMsgs(ulog);

% pixhawk system info
systeminfo = readSystemInformation(ulog);

% parameters
params = readParameters(ulog);

% topics we would like
topicList = {'actuator_controls_0','actuator_outputs','input_rc',...
             'airspeed_validated','vehicle_air_data',...
             'vehicle_gps_position','vehicle_global_position',...
             'vehicle_local_position','vehicle_attitude',...
             'vehicle_angular_acceleration','vehicle_angular_velocity',...
             'sensor_combined','rpm','airdata',...
             'estimator_status'};
         
% intersection of desired topics and available topics
topicsAvail = intersect(topicList,msg.TopicNames);

% read the topic messages and make the table accessible by row name
data = readTopicMsgs(ulog,'TopicNames',topicsAvail,...
                     'InstanceID',num2cell(zeros(size(topicsAvail))));
data.Properties.RowNames = data.TopicNames;

%% time synchronization

% we do all of the data re-timing based on GPS time, if
% vehicle_gps_position not available, throw error
if all(strcmp(topicsAvail,'vehicle_gps_position'))
    error('ulg2tt.m requires gps data for time synchronization');
    % TODO: make this an optional requirement and
    %       if gps not available, use PX4 time
end

% offset for t0 (not all topics logged right away)
t0_offset = 0.2; % s

% get the first gps reading to get timestamps
vehicle_gps_position = data('vehicle_gps_position',:).TopicMessages{:};
FlightTimeZulu = datetime(vehicle_gps_position.time_utc_usec(1),...
    'ConvertFrom','epochtime','TicksPerSecond',1e6);
FlightTimeZulu = FlightTimeZulu + seconds(t0_offset); % t0 offset
t0_PX4 = vehicle_gps_position.timestamp(1) + seconds(t0_offset); % t0 in PX4 time
tf_PX4 = vehicle_gps_position.timestamp(end); % tf in PX4 time
T_PX4 = seconds(tf_PX4 - t0_PX4); % approximate flight time

% important timestamps
[hour,minute,second] = hms(FlightTimeZulu);
t0_Z = duration(hour,minute,second,'Format','hh:mm:ss.SS'); % t0 in Zulu time

% desired time step and uniform time array
dt = 0.01; %s
Tu = floor(T_PX4/dt)*dt;
N = Tu/dt+1;
Time = seconds((0:dt:Tu).');

% GPS timestamps
ZuluTime = (t0_Z:seconds(dt):(t0_Z+seconds(Tu))).'; % uniform GPS timestamps
ZuluTime.Format = 'hh:mm:ss.SS'; % change format
TT = timetable(Time,ZuluTime); % create timetable

%% vehicle_<...> (Outputs of the EKF)

% vehicle_local_position gives
%   NED position
%   NED velocity
%   NED acceleration
%   heading
vehicle_local_position = data('vehicle_local_position',:).TopicMessages{:};
vehicle_local_position = vehicle_local_position(:,[4:6 9:11 15:17 18]);
vehicle_local_position.timestamp = seconds(seconds(vehicle_local_position.timestamp-t0_PX4)); % shift time
vehicle_local_position = retime(vehicle_local_position,Time,'pchip'); % retime
TT = addvars(TT,vehicle_local_position{:,1:3},...
                vehicle_local_position{:,4:6},...
                vehicle_local_position{:,7:9},...
                vehicle_local_position{:,10}*180/pi,...
                'NewVariableNames',{'NED_m','vi_m_s','ai_m_s2','heading_deg'});
TT{:,2} = TT{:,2} - TT{1,2}; % zero position data

% vehicle_global_position gives
%   Lat-Lon-Alt position in degrees and feet
%   esllipsoidal alititude in feet
%   terrain altitude in feet
vehicle_global_position = data('vehicle_global_position',:).TopicMessages{:};
vehicle_global_position = vehicle_global_position(:,[1:4 8]);
vehicle_global_position.timestamp = seconds(seconds(vehicle_global_position.timestamp-t0_PX4)); % shift time
vehicle_global_position = retime(vehicle_global_position,Time,'pchip'); % retime
LatLon_deg = [vehicle_global_position.lat, vehicle_global_position.lon];
Alt_ft = vehicle_global_position.alt/0.3048;
Alt_Ellipsoid_ft = vehicle_global_position.alt_ellipsoid/0.3048;
Alt_Terrain_ft = vehicle_global_position.terrain_alt/0.3048;
TT = addvars(TT,LatLon_deg,Alt_ft,Alt_Ellipsoid_ft,Alt_Terrain_ft); % add to timetable
     
% vehicle_attitude gives
%   attitude quaternion
%   rotation from NED to body frame
%   Euler Angles
vehicle_attitude = data('vehicle_attitude',:).TopicMessages{:}; % get timetable
vehicle_attitude.timestamp = seconds(seconds(vehicle_attitude.timestamp-t0_PX4)); % shift time
vehicle_attitude = retime(vehicle_attitude(:,1),Time,'pchip'); % retime
R_BI = quat2dcm(vehicle_attitude.q); % rotation from NED to body frame
[Yaw_rad,Pitch_rad,Roll_rad] = dcm2angle(R_BI,'ZYX'); % 3-2-1 euler parameterization
EulerAngles_deg = [Roll_rad,Pitch_rad,Yaw_rad]*180/pi; % euler angles in degrees
TT = addvars(TT,EulerAngles_deg,permute(R_BI,[3,1,2]),'NewVariableNames',{'EulerAngles_deg','R_BI'}); % add to timetable

%% estimator states
% in rare cases, the ECL EKF output predictor can diverge (in a spin)

% estimator_status gives
%   [1:4]   Quaternions
%   [5:7]   Velocity NED (m/s)
%   [8:10]  Position NED (m)
%   [11:13] IMU delta angle bias XYZ (rad)
%   [14:16] IMU delta velocity bias XYZ (m/s)
%   [17:19] Earth magnetic field NED (gauss)
%   [20:22] Body magnetic field XYZ (gauss)
%   [23:24] Wind velocity NE (m/s)
% Note: usually time offset of the EKF is 200ms
estimator_status = data('estimator_status',:).TopicMessages{:}; % get timetable
estimator_status.timestamp = seconds(seconds(estimator_status.timestamp-t0_PX4)); % shift time
estimator_status = retime(estimator_status(:,1),Time,'pchip'); % retime
TT = addvars(TT,estimator_status.states,'NewVariableNames',{'EstimatorStates'}); % add to timetable

%% other aircraft states

% From vehicle_attitude and vehicle_local_position we can compute
%   velocity and acceleration in body frame
%   derived airspeed data (airspeed, alpha, beta)
% Note: derived Velocity assumes no wind
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

% vehicle_angular_velocity gives
%   body angular velocity
vehicle_angular_velocity = data('vehicle_angular_velocity',:).TopicMessages{:};
gyroTimeShift = vehicle_angular_velocity.timestamp(1)-vehicle_angular_velocity.timestamp_sample(1); % gyro time shift
vehicle_angular_velocity.timestamp = seconds(seconds((vehicle_angular_velocity.timestamp-gyroTimeShift)-t0_PX4)); % shift time
vehicle_angular_velocity = retime(vehicle_angular_velocity(:,2),Time,'pchip'); % retime
TT = addvars(TT,vehicle_angular_velocity.xyz*180/pi,'NewVariableNames',{'omega_deg_s'}); % add to timetable
   
% unwrap yaw
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

%% actuators and RC

% RC inputs
actuator_controls = data('actuator_controls_0',:).TopicMessages{:};
actuator_controls.timestamp = seconds(seconds(actuator_controls.timestamp-t0_PX4)); % shift time
actuator_controls = retime(actuator_controls(:,2),Time,'nearest'); % retime
rc_in_PWM = actuator_controls.control;
TT = addvars(TT,rc_in_PWM,'NewVariableNames',{'rc_in_PWM'}); % add to timetable
    
% actuator outputs
actuator_outputs = data('actuator_outputs',:).TopicMessages{:};
actuator_outputs.timestamp = seconds(seconds(actuator_outputs.timestamp-t0_PX4)); % shift time
actuator_outputs = retime(actuator_outputs(:,2),Time,'nearest'); % retime
TT = addvars(TT,actuator_outputs.output(:,[1 2 4]),actuator_outputs.output(:,3),'NewVariableNames',{'deltaPWM_us','ThrottlePWM_us'}); % add to timetable

%% air data

% airspeed_validated gives
%   equivalent airspeed
%   true airspeed
if any(strcmp(topicsAvail,'airspeed_validated'))
    airspeed_validated = data('airspeed_validated',:).TopicMessages{:};
    for k = 1:height(airspeed_validated) % remove invalid measurements
        if airspeed_validated.airspeed_sensor_measurement_valid(k) == 0
            airspeed_validated{k,:} = nan(size(airspeed_validated{k,:}));
        end
    end
    airspeed_validated.timestamp = seconds(seconds(airspeed_validated.timestamp-t0_PX4)); % shift time
    airspeed_validated = retime(airspeed_validated(:,1:5),Time,'pchip'); % retime
    EAS_m_s = airspeed_validated.equivalent_airspeed_m_s;
    TAS_m_s = airspeed_validated.true_airspeed_m_s;
    TT = addvars(TT,EAS_m_s,TAS_m_s,'NewVariableNames',{'EquivalentAirSpeed_m_s','TrueAirSpeed_m_s'}); % add to timetable
end
     
% airdata gives
%   measured alpha, beta, V from a vaned air data unit
% Note: this is a custom topic that must be defined in PX4
if any(strcmp(topicsAvail,'airdata'))
    airdata = data('airdata',:).TopicMessages{:};
    airdata.timestamp = seconds(seconds(airdata.timestamp-t0_PX4)); % shift time
    airdata = retime(airdata(:,1:5),Time,'pchip'); % retime
    airspeed_kts = airdata.airspeed_kts;
    alpha_deg = airdata.alpha_deg;
    beta_deg = airdata.beta_deg;
    TT = addvars(TT,airspeed_kts,alpha_deg,beta_deg,'NewVariableNames',{'airspeed_kts','alpha_deg_measured','beta_deg_measured'}); % add to timetable
end

%% add data to timetable and save data to mat file

% ulog filename
[~,filename,ext] = fileparts(ulogPath);
ULOGfilename = [filename ext];

% add important metadata to table
TT = addprop(TT,'Source','table');
TT = addprop(TT,'AvailableTopics','table');
TT = addprop(TT,'Parameters','table');
TT = addprop(TT,'SystemInfo','table');
TT = addprop(TT,'FlightTimestamp','table');
TT.Properties.CustomProperties.Source = ULOGfilename;
TT.Properties.CustomProperties.AvailableTopics = msg;
TT.Properties.CustomProperties.Parameters = params;
TT.Properties.CustomProperties.SystemInfo = systeminfo;
TT.Properties.CustomProperties.FlightTimestamp = FlightTimeZulu;

end