% Example usage of ulg2tt.m
%
% (C) 2021  Jeremy Hopwood <jeremyhopwood@vt.edu>
%           James Gresham 
%
% See DOI: TODO
%
close all
clear
clc

% convert ulog file to a timetable
ulogPath = '../../20211116/20211116_CZ1501_Flt4.ulg';
data = ulg2tt(ulogPath);

% custom options
opts = ulg2ttOptions;
opts.MessageSet = 'Debugging'
opts.TimeStep = 0.2;
opts.Plot = true;
data = ulg2tt(ulogPath,ulg2ttOptions);

% save the timetable in a .mat file in same location as the ulog
[filepath,filename,~] = fileparts(ulogPath);
MATfilename = [filepath '/' filename '.mat'];
% save(MATfilename,'data');

% Get properties of the data. For example,
data.Properties
data.Properties.CustomProperties
VariableNames = data.Properties.VariableNames.'
SampleRate = data.Properties.SampleRate

% get state variable for entire flight
time_vec = seconds(data.Time);
NED_m = data.NED_m;

% trim timetable for a time range and get state variable
S = timerange(seconds(259),seconds(290));
maneuver = data(S,:);
time_vec_maneuver = seconds(maneuver.Time);
delta_PWM_maneuver = maneuver.deltaPWM_us;

% plot select variables from timetable
figure
stackedplot(maneuver,{'EulerAngles_deg','V_m_s_derived','alpha_deg_derived',...
    'beta_deg_derived','omega_deg_s','deltaPWM_us'})

% retime the data
dt_new = seconds(0.1);
maneuver_downsample = retime(maneuver,'regular','linear','TimeStep',dt_new);

