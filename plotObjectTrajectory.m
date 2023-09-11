function plotObjectTrajectory(translations,rotations,scaleFactor,numObjects,stl)
%
% Copyright (c) 2023 Jeremy W. Hopwood. All rights reserved.
%
% This function ...
%
% Requirements: UAV Toolbox
%
% Inputs:
%
%   translations  See documentation for plotTransforms.m
%
%   rotations     See documentation for plotTransforms.m
%

% input arguments error checking
if nargin~=5
    error('Incorrect number of inputs.');
end
if (length(translations)~=length(rotations))
    error('Number of samples is not consistent.');
end

% color of path
BurntOrange = [232,119,34]/255;
pathColor = BurntOrange;

% NED --> ENU
ENU = [translations(:,2), translations(:,1), -translations(:,3)];
x = ENU(:,1);
y = ENU(:,2);
z = ENU(:,3);

% number of samples
N = length(translations);

% plot the object path
hold on
plot3(x,y,z,'linewidth',1.5,'Color',pathColor)

% plot the objects
samplingFactor = floor(N/(numObjects-1));
indices = 1:samplingFactor:N;
for k = indices
    plotTransforms(ENU(k,:),rotations(k,:),'MeshFilePath',stl,'FrameSize',scaleFactor)
end
hold off
grid on

% Axis labels and legend
xlabel('East [m]','FontSize',10)
ylabel('North [m]','FontSize',10)
zlabel('Altitude [m]','FontSize',10)

% set the axes limits
limits = [min(x),max(x),min(y),max(y),min(z),max(z)];
limits = limits + scaleFactor*[-1,1,-1,1,-1,1];
axis(limits);

% set lighting, view, aspect ratio, etc.
grid on
view(35,25);
daspect([1 1 1]);
lightangle(35,60)