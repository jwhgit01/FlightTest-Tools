% Example usage of ulg2pose.m and trajectory.m
%
% (C) 2021  Jeremy Hopwood <jeremyhopwood@vt.edu>
%
close all
clear
clc

% convert ulog to pose data (position and attitude)
ulogPath = '../../20211116/20211116_CZ1501_Flt4.ulg';
dt = 0.05;
timeVec = 75:dt:95;
[north,east,down,roll,pitch,yaw] = ulg2pose(ulogPath,timeVec);

%% Plot the trajectory of the aircraft
clc
close all

% options
aircraft = 'cessna';
scaleFactor = 10; % sizing factor of the aircraft 
numAircraft = 10; % number of aircraft to plot along the path

% plot
trajectory(north,east,down,roll,pitch,yaw,scaleFactor,numAircraft,aircraft)
xlabel('\Delta East [m]')
ylabel('\Delta North [m]')
zlabel('Altitude [m]')

%% Animate the trajectory of the aircraft
clc
close all

% aircraft options
scaleFactor = 10;
aircraft = 'cessna';

% make the figure fairly large for good resolution
figure('Position',[50 50 720 720])

% create the VideoWriter object
videoObject = VideoWriter('myAnimation.mp4','MPEG-4');
videoObject.FrameRate = 1/dt; % real-time

% animate and save
trajectory(north,east,down,roll,pitch,yaw,scaleFactor,videoObject,aircraft)


