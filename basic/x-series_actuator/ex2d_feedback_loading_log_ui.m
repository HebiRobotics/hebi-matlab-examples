% Load a saved log file with feedback from an actuator and plot some of 
% the data.
%
% HEBI Robotics
% July 2018

clear *;
close all;

% Bring up a dialog box to interactively choose log file(s) to load
logs = HebiUtils.loadGroupLogsUI();

% Select the first log for plotting
log = logs{1};

% Plot using some handy helper functions
HebiUtils.plotLogs( log, 'position', 'figNum', 101 );
HebiUtils.plotLogs( log, 'velocity', 'figNum', 102 );
HebiUtils.plotLogs( log, 'effort', 'figNum', 103 );


