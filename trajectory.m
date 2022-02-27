function trajectory(north,east,down,roll,pitch,yaw,scaleFactor,var,aircraft)
%
% (2021) Jeremy Hopwood <jeremyhopwood@vt.edu>
%
% Inspired by:
%   Valerio Scordamaglia (2021). Trajectory and Attitude Plot Version 3
%   (https://www.mathworks.com/matlabcentral/fileexchange/5656-trajectory-and-attitude-plot-version-3),
%   MATLAB Central File Exchange.
%
% Inputs:
%   north,east,down  NED position                   [N x 1]
%   roll,pitch,yaw   Euler angles (rad)             [N x 1]
%   scale_factor     body aircraft dimension factor [scalar]
%   var              number of aircraft to plot     [integer]
%                or  VideoWriter object
%   aircraft         select the aircraft model      [string]
%                           A-10        A-10             
%                           cessna      Cessna
%                           mig         Mig
%                           tomcat      Tomcat
%                           jet         Generic jet
%                           shuttle     Space Shuttle
%                           helicopter  Helicopter
%                           B747        Boeing 747
%                           biplane     Generic Biplane
%                           md90        Md90
%                           dc10        Dc10
%                           ah64        Ah64 helicopter
%                           gripen      JAS 39 Gripen


% input arguments error checking
if nargin~=9
    error('Incorrect number of inputs.');
end
if (length(north)~=length(east))||(length(north)~=length(down))||(length(east)~=length(down))
    error('Number of position samples is not consistent.');
end
if (length(pitch)~=length(roll))||(length(pitch)~=length(yaw))||(length(roll)~=length(yaw))
    error('Number of attitude samples is not consistent.');
end
if length(pitch)~=length(north)
    error('Number of position samples is not consitient with the number of attitude samples.');
end

% number of samples
N = length(north);

% input, var, can either be a video object or a number
samplingFactor = 0;
if isnumeric(var)
    video = false;
    samplingFactor = floor(N/(var-1));
    if var > length(north) 
        error('Number of aircraft is greater than the number of data points!');
    end
else
    try
        var.FrameRate;
    catch
        error('Input "var" must be a number or VideoWriter object.');
    end
    video = true;
end

% load aircraft mat file
func_path = which('trajectory.m');
aircraft_dir = replace(func_path,'trajectory.m','aircraft');
try
    load([aircraft_dir '/' aircraft],'V','F','C');
catch
    warning(['Warning: ',aircraft,' not found. Default = cessna']);
    load([aircraft_dir '/cessna'],'V','F','C');
end

% NED -> ENU
x = east; y = north; z = -down;

% correction and scaling of aircraft models
correctionFactor = max(abs(V(:,1)));
V = V./(correctionFactor/scaleFactor);

% plot an aircraft at the first point
hold on
Vnew = transformVertices(x(1),y(1),z(1),roll(1),pitch(1),yaw(1),V);
p = newPatch(Vnew,F,C);
fig = gcf;

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
if video
    open(var);
    frame = getframe(fig);
    writeVideo(var,frame);
end

% color of path
BurntOrange = [232,119,34]/255;
pathColor = BurntOrange;

% loop through rest of data
for ii = 2:N
    
    % plot segment of path
    plot3([x(ii-1),x(ii)],[y(ii-1),y(ii)],[z(ii-1),z(ii)],'linewidth',1.5,'Color',pathColor);
    
    % if we are making a video
    if video
       
        Vnew = transformVertices(x(ii),y(ii),z(ii),roll(ii),pitch(ii),yaw(ii),V);
        p.Vertices = Vnew;
        frame = getframe(fig);
        writeVideo(var,frame);   
    
    % or if we should otherwise plot an aircraft
    elseif mod(ii,samplingFactor)==0
       
        Vnew = transformVertices(x(ii),y(ii),z(ii),roll(ii),pitch(ii),yaw(ii),V);
        p = newPatch(Vnew,F,C);
        
    end
    
end

% if video, close object
if video
    close(var);
end

end

% transformation of aircraft patch data to ENU frame
function Vnew = transformVertices(x,y,z,roll,pitch,yaw,V)
    RIB = [cos(pitch)*cos(yaw),cos(yaw)*sin(pitch)*sin(roll)-cos(roll)*sin(yaw),sin(roll)*sin(yaw)+cos(roll)*cos(yaw)*sin(pitch);
           cos(pitch)*sin(yaw),cos(roll)*cos(yaw)+sin(pitch)*sin(roll)*sin(yaw),cos(roll)*sin(pitch)*sin(yaw)-cos(yaw)*sin(roll);
           -sin(pitch),cos(pitch)*sin(roll),cos(pitch)*cos(roll)];
    R1 = diag([1,1,-1]); % correction before transformation to body frame
    R2 = [0,1,0;1,0,0;0,0,-1]; % correction after transformation to body frame
    Vnew = (R2*RIB*R1*V.').';
    X0 = repmat([x y z], size(Vnew,1), 1);
    Vnew = Vnew + X0;
end

% plot a new aircraft body
function p = newPatch(V,F,C)
    ChicagoMaroon = [134,31,65]/255;
    p=patch('faces', F, 'vertices' ,V);          
    p.FaceVertexCData = C;  
    p.EdgeColor = 'none'; 
    p.FaceColor = ChicagoMaroon;
end
