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

%% Setup Group
clear *;
close all;
HebiLookup.initialize();

familyName = 'Test Family';
moduleNames = 'Test Actuator';
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

%% Blocking Trajectory
trajGen = HebiTrajectoryGenerator();
group.startLog( 'dir', 'logs' );

% Go from 0 to 180-degrees in 3 seconds
waypoints = [
    0;
    2*pi ];
time = [ 0 6 ];

% This function generates smooth minimum jerk trajectories
trajectory = trajGen.newJointMove( waypoints, 'time', time );

% Visualize the trajectory
HebiUtils.plotTrajectory(trajectory);
drawnow;

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

% Stop logging and plot the velocity data using helper functions
log = group.stopLog();
HebiUtils.plotLogs( log, 'position', 'figNum', 101 );
HebiUtils.plotLogs( log, 'velocity', 'figNum', 102 );
