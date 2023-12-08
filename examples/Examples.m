%% Example usage of ulg2tt.m
close all
clc

ulogPath = '../../20211116/20211116_CZ1501_Flt4.ulg';

opts = ulg2ttOptions;
opts.TimeStep = 0.05;
opts.MessageSet = 'Advanced';
opts.Plot = true;

data = ulg2tt(ulogPath,opts);

NED_m = data.NED_m;
EulerAnlges_rad = data.EulerAnlges_rad;

%% Example usage of ulg2pose.m
close all
clc

ulogPath = '../../20211116/20211116_CZ1501_Flt4.ulg';

dt = 0.05;
timeVec = 75:dt:95;

[north,east,down,roll,pitch,yaw] = ulg2pose(ulogPath,timeVec);

%% Example usage of trajectory.m for plotting
close all
clc

% options
aircraft = 'cessna';
scaleFactor = 10; % sizing factor of the aircraft 
numAircraft = 10; % number of aircraft to plot along the path

% plot
trajectory(north,east,down,roll,pitch,yaw,scaleFactor,numAircraft,aircraft)
xlabel('\Delta East [m]')
ylabel('\Delta North [m]')
zlabel('Altitude [m]')

%% Example usage of trajectory.m for animation
close all
clc

% aircraft options
scaleFactor = 10;
aircraft = 'cessna';

% make the figure fairly large for good resolution
figure('Position',[50 50 720 720])

% create the VideoWriter object
videoObject = VideoWriter('trajectoryAnimation.mp4','MPEG-4');
videoObject.FrameRate = 1/dt; % real-time

% animate and save
trajectory(north,east,down,roll,pitch,yaw,scaleFactor,videoObject,aircraft)

%% Example usage of animateObject.m
close all
clc

% NED position
translations = NED_m;

% convert Euler angles to quaternions (or SO(3) objects)
rotations = eul2quat(EulerAnlges_rad,'XYZ');

% aircraft options
scaleFactor = 10;
STL = 'quad.stl';

% make the figure fairly large for good resolution
figure('Position',[50 50 720 720])

% create the VideoWriter object
videoObject = VideoWriter('animateObjectAnimation.mp4','MPEG-4');
videoObject.FrameRate = 1/dt; % real-time

animateObject(translations,rotations,scaleFactor,videoObject,STL)

