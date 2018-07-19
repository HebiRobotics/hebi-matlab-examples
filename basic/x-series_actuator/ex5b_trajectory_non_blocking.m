% Generate a trajectory and execute it using the non-blocking API.  This
% code does the exact same commands as the previous blocking API example.
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

%% Non-Blocking Trajectory
trajGen = HebiTrajectoryGenerator();
cmd = CommandStruct();
group.startLog( 'dir', 'logs' );

% Go from 0 to 180-degrees in 3 seconds
waypoints = [
    0;
    pi ];
time = [ 0 3 ];

% This function generates smooth minimum jerk trajectories
trajectory = trajGen.newJointMove( waypoints, 'time', time );

% Visualize the trajectory
HebiUtils.plotTrajectory(trajectory);
drawnow;

% This example executes the trajectory using the 'non-blocking' API.  In
% this format, we have to make a while loop to step through the trajectory
% at each timestep, evaluate the trajectory to get commands, and send them
% to the actuator.  This makes the top-level code more complex relative to
% the blocking example, but it has the advantage that it is easy to 
% evaluate feedback every timestep and change the trajectory or some other 
% behavior
t0 = tic();
t = toc(t0);
while t < trajectory.getDuration()
    
    fbk = group.getNextFeedback();
    
    % Get trajectory state at the current time
    t = toc(t0);
    [pos, vel, accel] = trajectory.getState(t);
    
    % Send the commands
    cmd.position = pos;
    cmd.velocity = vel;
    group.send(cmd);
    
end

% Reverse the waypoints and to go back to the first position
waypoints = flipud(waypoints);
trajectory = trajGen.newJointMove( waypoints, 'time', time );

t0 = tic();
t = toc(t0);
while t < trajectory.getDuration()
    
    fbk = group.getNextFeedback();
    
    % Get trajectory state at the current time
    t = toc(t0);
    [pos, vel, accel] = trajectory.getState(t);
    
    % Send the commands
    cmd.position = pos;
    cmd.velocity = vel;
    group.send(cmd);
    
end

% Stop logging and plot the velocity data using helper functions
log = group.stopLog();
HebiUtils.plotLogs( log, 'position', 'figNum', 101 );
HebiUtils.plotLogs( log, 'velocity', 'figNum', 102 );

