% Load a saved log file with feedback from an actuator and plot some of 
% the data.
%
% HEBI Robotics
% July 2018

clear *;
close all;

logFileFolder = '/logs/';
logFileName = 'exampleLog.hebilog';     % the '.hebilog' is optional

log = HebiUtils.loadGroupLog( [logFileFolder logFileName] );

% Plot using some handy helper functions
HebiUtils.plotLogs( log, 'position', 'figNum', 101 );
HebiUtils.plotLogs( log, 'velocity', 'figNum', 102 );
HebiUtils.plotLogs( log, 'effort', 'figNum', 103 );


