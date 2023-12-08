% Example usage of FlightData objects
%
% (C) 2023 Jeremy Hopwood <jeremyhopwood@vt.edu>
%
addpath(genpath('../src/'))
close all
clear
clc

% convert ulog file to a timetable
ulogFile = 'ExampleData.ulg';

% Create a FlightData object
fd = FlightData(ulogFile);
fd.MessageSet = 'Estimation';
fd.CutoffFrequency = 10;

% Convert ulog file to a timetable
fd.ulg2tt;
data = fd.Data;

% Trim the data to a timerange of interest
ts = seconds(115);
te = seconds(565);
trange = timerange(ts,te);
maneuver = data(trange,:);

% Plot
figure
stackedplot(maneuver,{'NED_m','EulerAngles_rad','vb_m_s','omega_rad_s'})
