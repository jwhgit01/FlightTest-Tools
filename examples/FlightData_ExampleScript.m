% Example usage of FlightData objects
%
% (C) 2024 Jeremy Hopwood <jeremyhopwood@vt.edu>
%
addpath(genpath('../src/'))
close all
clear
clc

% Create a FlightData object
fd = FlightData("ExampleData.ulg");
fd.Estimation = true;
fd.CutoffFrequency = 10;

% Convert ulog file to a timetable
fd.ulg2tt;
data = fd.Data;

% Plot data from entire flight
figure
fd.plot;

% Trim the data to a timerange of interest
ts = seconds(140);
te = seconds(150);
trange = timerange(ts,te);
maneuver = data(trange,:);

% Plot the maneuver
figure
stackedplot(maneuver,{'NED_m','EulerAngles_rad','vb_m_s','omega_rad_s'})
