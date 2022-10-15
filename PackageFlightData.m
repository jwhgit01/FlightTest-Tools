%% Load flight data and choose maneuver times
% run this section first
close all
clear
clc

% read ulog file
FlightUID = '20220928_MTD2_Flt2';
MATFileName = ['HinfinityFilter_' FlightUID];
ulogFile = ['G:\My Drive\Research\FlightTest\20220928_MTD\' FlightUID '.ulg'];
flightdata = ulg2tt(ulogFile);

% plot flight and select timestamps
figure
stackedplot(flightdata,{'NED_m','EulerAngles_deg','vb_m_s','TrueAirSpeed_m_s','rpm_indicated'})

%% Selected maneuvers

% choose from stackedplot above, then run here to end
timeranges = [530 570;
              617 650;
              702 752;
              814 862];

%% Process data (filter, wind reconstruction, etc.)

% load MTD
addpath(genpath('G:\My Drive\Research\SystemIdentification\SysID-Tools'));
load('G:\My Drive\Research\SystemIdentification\MTD2\MTD2.mat','MTD2');

% loop through maneuvers
for ii = 1:size(timeranges,1)
    
    % trim timetable
    t0 = timeranges(ii,1);
    t1 = timeranges(ii,2);
    S = timerange(seconds(t0),seconds(t1));
    maneuver = flightdata(S,:);
    maneuverUID = ['m' FlightUID '_s' num2str(floor(t0)) '_e' num2str(floor(t1))];
    N = height(maneuver);
    t = seconds(maneuver.Time);

    % position
    NED_m = maneuver.NED_m;

    % attitude
    EulerAngles_rad = maneuver.EulerAngles_deg*pi/180;

    % inertial velocity in inertial and body frames from PX4 EKF
    vi_m_s = maneuver.vi_m_s;
    vb_m_s = maneuver.vb_m_s;

    % angular velocity
    omega_rad_s = maneuver.omega_deg_s*pi/180;

    % actuator deflections in rad
    delta_raw = maneuver.actuator_outputs_RAW(:,[1 2 4]);
    da_rad_cmd = MTD2.ActuatorModel.StaticMap.da.Map(delta_raw(:,1));
    de_rad_cmd = MTD2.ActuatorModel.StaticMap.de.Map(delta_raw(:,2));
    dr_rad_cmd = MTD2.ActuatorModel.StaticMap.dr.Map(delta_raw(:,3));

    % remove actuator delay
    da_rad_inst = MTD2.ActuatorModel.TimeDelay.da.Map(t,da_rad_cmd);
    de_rad_inst = MTD2.ActuatorModel.TimeDelay.de.Map(t,de_rad_cmd);
    dr_rad_inst = MTD2.ActuatorModel.TimeDelay.dr.Map(t,dr_rad_cmd);

    % account for rate limit
    da_rad = MTD2.ActuatorModel.RateLimit.da.Map(t,da_rad_inst);
    de_rad = MTD2.ActuatorModel.RateLimit.de.Map(t,de_rad_inst);
    dr_rad = MTD2.ActuatorModel.RateLimit.dr.Map(t,dr_rad_inst);

    % rpm measurements
    N_poles = 12;
    rpm = zeros(N,1);
    rpm(1) = maneuver.rpm_indicated(1)/N_poles;
    for k = 2:N
        rpm_ind = maneuver.rpm_indicated(k)/N_poles;
        if rpm_ind > 1e5 % remove numerical outliers
            rpm(k) = rpm(k-1);
        else
            rpm(k) = rpm_ind;
        end
    end

    % reconstruct wind from ADU measurements
    V_ADU = maneuver.TrueAirSpeed_m_s;
    alpha_vane = maneuver.alpha_deg_measured*pi/180;
    beta_vane = maneuver.beta_deg_measured*pi/180;
    vr_true_m_s = MTD2.Sensors.NSLADU1.InverseModel(V_ADU,beta_vane,alpha_vane,omega_rad_s);
    w_true_m_s = zeros(N,3);
    for k = 1:N
        R_IB = squeeze(maneuver.R_BI(k,:,:)).';
        w_true_m_s(k,:) = vi_m_s(k,:) - (R_IB*vr_true_m_s(k,:).').';
    end
    
    % put maneuver in a timetable
    data.(maneuverUID) = timetable(seconds(t),NED_m,EulerAngles_rad,vb_m_s,omega_rad_s,da_rad,de_rad,dr_rad,rpm,vr_true_m_s,w_true_m_s);

end

figure
hold on
plot(t,da_rad_cmd,'r')
plot(t,da_rad_inst,'g')
plot(t,da_rad,'b')
hold off
grid on


%% Save to mat file

save('G:\My Drive\Research\WindEstimation\H-infinity Filter\' MATFileName '.mat','data')
