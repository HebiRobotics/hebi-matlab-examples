% Generate a trajectory and execute it using the non-blocking API.  This
% code does the exact same commands as the previous blocking API example.
%
% For more information type:
%    help HebiTrajectoryGenerator
%    help HebiTrajectory
%
% This script assumes that you have a group created with 1 module in it.
%
% HEBI Robotics
% June 2018

clear *;
close all;

HebiLookup.initialize();

familyName = 'My Family';
moduleNames = 'Test Module';  
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

cmd = CommandStruct();

exampleDuration = 10; % sec
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

% This function executes the trajectory using the 'non-blocking' API.  In
% this format, we have to make a while loop to step through the trajectory
% at each timestep, evaluate the trajectory to get commands, and send them
% to the actuator.  This makes the top-level code more complex, but it has
% the advantage that it is easy to evaluate feedback every timestep and
% change the trajectory or some other behavior
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

log = group.stopLog(); 

% Plot using some handy helper functions
HebiUtils.plotLogs( log, 'position', 'figNum', 101 );
HebiUtils.plotLogs( log, 'velocity', 'figNum', 102 );

