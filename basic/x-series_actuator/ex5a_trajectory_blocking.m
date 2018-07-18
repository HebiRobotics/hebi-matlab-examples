% Generate a trajectory and execute it using the blocking API.  This code
% does the exact same commands as the next non-blocking API example.
%
% For more information type:
%    help HebiTrajectoryGenerator
%    help HebiTrajectory
%
% This script assumes you can create a group with 1 module.
%
% HEBI Robotics
% June 2018

%%
clear *;
close all;

HebiLookup.initialize();

familyName = 'My Family';
moduleNames = 'Test Module';  
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

cmd = CommandStruct();

exampleDuration = 10;   % [sec]
exampleTimer = tic;

group.startLog( 'dir', 'logs' );

trajGen = HebiTrajectoryGenerator();

% Go from 0 to 180-degrees in 3 seconds
waypoints = [ 0; 
              pi ];
time = [ 0 3 ];

trajectory = trajGen.newJointMove( waypoints, 'time', time );

% Visualize the trajectory
HebiUtils.plotTrajectory(trajectory);

% This function executes the trajectory using the 'blocking' API.  This
% means that your program will go into this function and 'block' the rest
% of the code from running until it is done.  This makes the high-level
% code convenient, but it makes it more difficult to grab and analyze
% feedback while the motion is being executed.
trajGen.executeTrajectory( group, trajectory );

% Reverse the waypoints and to go back to the first position
waypoints = flipud(waypoints);
trajectory = trajGen.newJointMove( waypoints, 'time', time );
trajGen.executeTrajectory( group, trajectory );

log = group.stopLog(); 

% Plot using some handy helper functions
HebiUtils.plotLogs( log, 'position', 'figNum', 101 );
HebiUtils.plotLogs( log, 'velocity', 'figNum', 102 );

