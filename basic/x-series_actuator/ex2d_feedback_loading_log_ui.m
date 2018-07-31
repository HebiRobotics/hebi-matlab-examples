% Load a saved log file with feedback from an actuator and plot some of 
% the data.
%
% For more information type:
%    help HebiUtils
%    help HebiGroup
%
% HEBI Robotics
% July 2018

%%
clear *;
close all;

% Bring up a dialog box to interactively choose log file(s) to load.  Note
% that when using the UI to select logs they get returned as a cell array.
logs = HebiUtils.loadGroupLogsUI();

% Plot using some handy helper functions
HebiUtils.plotLogs( logs, 'position', 'figNum', 101 );
HebiUtils.plotLogs( logs, 'velocity', 'figNum', 102 );
HebiUtils.plotLogs( logs, 'effort', 'figNum', 103 );
