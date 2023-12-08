function animateObject(translations,rotations,scaleFactor,vidObj,stl)
%animateObject 
%
% Copyright (c) 2023 Jeremy W. Hopwood. All rights reserved.
%
% This function animates the trajectory of a rigid body. 
%
% Example usage:
%   fig = figure;
%   fig.Position(3:4) = [1080 1080]); % resolution
%   vidObj = VideoWriter('myAnimation.mp4','MPEG-4');
%   vidObj.FrameRate = 1/dt; % real-time
%   animateObject(translations,rotations,1,vidObj,'quad.stl')
%
% Requirements: UAV Toolbox
%
% Inputs:
%
%   translations  See documentation for plotTransforms.m
%
%   rotations     See documentation for plotTransforms.m
%  
%   scaleFactor   A scale factor that determines how large the 3D object
%                 defined by the input 'stl' is displayed.
%
%   vidObj        The number of 3D objects to plot along the path.
%
%   stl           The filename of an STL file that defines the 3D object.
%


% input arguments error checking
if nargin~=5
    error('Incorrect number of inputs.');
end
if (length(translations)~=length(rotations))
    error('Number of samples is not consistent.');
end

% NED --> ENU
ENU = [translations(:,2), translations(:,1), -translations(:,3)];
x = ENU(:,1);
y = ENU(:,2);
z = ENU(:,3);

% number of samples
N = length(translations);

% plot an aircraft at the first point
hold on
ax = plotTransforms(ENU(1,:),rotations(1,:),'MeshFilePath',stl,'FrameSize',scaleFactor);
fig = gcf;
xlabel('East [m]','FontSize',18)
ylabel('North [m]','FontSize',18)
zlabel('$\Delta h$ [m]','FontSize',18,'Interpreter','latex')

% set the axes limits
limits = [min(x),max(x),min(y),max(y),min(z),max(z)];
limits = limits + scaleFactor*[-1,1,-1,1,-1,1];
axis(limits);

% set lighting, view, aspect ratio, etc.
grid on
view(35,25);
daspect([1 1 1]);
lightangle(35,60)

% if video, open and get and write first frame
frame = getframe(fig);
if ~isempty(vidObj)
    open(vidObj);
    writeVideo(vidObj,frame);
end
delete(ax.Children(2));

% color of path
BurntOrange = [232,119,34]/255;
pathColor = BurntOrange;

% loop through rest of data
for ii = 2:N

    % plot segment of path
    plot3([x(ii-1),x(ii)],[y(ii-1),y(ii)],[z(ii-1),z(ii)],'linewidth',1.5,'Color',pathColor);
    
    % if we are making a video
    ax = plotTransforms(ENU(ii,:),rotations(ii,:),'MeshFilePath',stl,'FrameSize',scaleFactor);
    frame = getframe(fig);
    if ~isempty(vidObj)
        writeVideo(vidObj,frame);
    end
    if ii < N
        delete(ax.Children(1));
    end

end

% if video, close object
if ~isempty(vidObj)
    close(vidObj);
end


end