function plotObjectTrajectory(NED,R_IB,scaleFactor,numObjects,stl,R_BS)
%plotObjectTrajectory
%
% Copyright (c) 2023 Jeremy W. Hopwood. All rights reserved.
%
% This function plots the trajectory of a rigid body. It displays the path
% as well as a number of 3D models depicting attitude.
%
% Requirements: UAV Toolbox
%
% Inputs:
%
%   NED           Array of position vectors
%
%   R_IB          Array of so(3) objects that map the body frame to the
%                 local NED frame
%
%   scaleFactor   A scale factor that determines how large the 3D object
%                 defined by the input 'stl' is displayed.
%
%   numObjects    The number of 3D objects to plot along the path.
%
%   stl           The filename of an STL file that defines the 3D object.
%
%   R_BS          The so(3) object that defines the orientation of the body
%                 in the STL file with respect to the body frame. (Optional)
%
%

% input arguments error checking
if nargin~=6
    error('Incorrect number of inputs.');
end
if (length(NED)~=length(R_IB))
    error('Number of samples is not consistent.');
end

% Plotting colors
pathColor = 'k';
objectColor = '#E66100';

% NED --> ENU
ENU = [NED(:,2), NED(:,1), -NED(:,3)];
x = ENU(:,1);
y = ENU(:,2);
z = ENU(:,3);

% Transform body
R_EI = so3([pi/2 0 pi],"eul"); % "E" denotes the ENU frame
R_RE = so3([0,0,pi],"eul"); % "R" denotes the robotics toolbox z-up frame
R_RS = R_RE*R_EI*R_IB*R_BS; % "S" denotes the STL frame

% number of samples
N = length(NED);

% plot the object path
hold on
plot3(x,y,z,'linewidth',1.5,'Color',pathColor)

% plot the objects
samplingFactor = floor(N/(numObjects-1));
k = 1:samplingFactor:N;
if isempty(stl)
    plotTransforms([x(k) -y(k) -z(k)],R_RS(:,k),'FrameSize',scaleFactor,"InertialZDirection","Down")
else
    plotTransforms([x(k) -y(k) -z(k)],R_RS(:,k),"MeshColor",objectColor,'MeshFilePath',stl,'FrameSize',scaleFactor,"InertialZDirection","Down")
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

end
