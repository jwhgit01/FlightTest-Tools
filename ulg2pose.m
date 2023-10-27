function [x_m,y_m,z_m,roll_rad,pitch_rad,yaw_rad,t_s] = ulg2pose(ulogPath,timeVec)
%ulg2pose
%
% (C) 2021 Jeremy Hopwood <jeremyhopwood@vt.edu>
%
% This function obtains position and attitude data from a ULOG file.
%
% Matlab Requirements:  Matlab R2020b or newer
%                       UAV Toolbox
%
% Inputs:
%   ulogPath  filepath to .ulg file
%   timeVec   array of PX4 timestamps at which to return data (optional)
% 
% Outputs:
%   x_m, y_m, z_m               North,East,Down position in meters
%   roll_rad,pitch_rad,yaw_rad  Euler angles in radians
%   t_s                         time vector in seconds

% required topics
topics = {'vehicle_local_position','vehicle_attitude'};

% read ulog file
ulog = ulogreader(ulogPath);

% flight time and duration
px4StartTime = ulog.StartTime;
px4EndTime = ulog.EndTime;
t0 = seconds(px4StartTime);
tf = seconds(px4EndTime);
if nargin < 2 % if no time vec is given, default
    timeVec = (0:0.1:(tf-t0)).';
end

% read the topic messages and make the table accessible by row name
data = readTopicMsgs(ulog,'TopicNames',topics,...
                     'InstanceID',num2cell(zeros(size(topics))));
data.Properties.RowNames = data.TopicNames;

% local position
vehicle_local_position = data('vehicle_local_position',:).TopicMessages{:};
vehicle_local_position.timestamp = seconds(seconds(vehicle_local_position.timestamp)-t0);
NED = retime(vehicle_local_position(:,4:6),seconds(timeVec),'pchip');

% attitude quaternion
vehicle_attitude = data('vehicle_attitude',:).TopicMessages{:};
vehicle_attitude.timestamp = seconds(seconds(vehicle_attitude.timestamp)-t0);
vehicle_attitude = retime(vehicle_attitude(:,1),seconds(timeVec),'pchip');

% quat 2 euler angles
q = vehicle_attitude.q;
qw = q(:,1); qx = q(:,2); qy = q(:,3); qz = q(:,4);
aSinInput = 2*(qw.*qy-qx.*qz);
aSinInput(aSinInput > 1) = 1;
aSinInput(aSinInput < -1) = -1;
eul = [atan2( 2*(qw.*qx+qy.*qz), 1-2*(qx.^2+qy.^2) ), ...
       asin( aSinInput ), ...
       atan2( 2*(qw.*qz+qx.*qy), 1-2*(qy.^2+qz.^2) )];
    
% assign outputs
x_m = NED{:,1};
y_m = NED{:,2};
z_m = NED{:,3};
roll_rad = eul(:,1);
pitch_rad = eul(:,2);
yaw_rad = eul(:,3);
t_s = timeVec;

end