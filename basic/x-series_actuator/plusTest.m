%% Setup Group
clear *;
close all;
HebiLookup.initialize();

familyName = 'Plus_Test';
moduleNames = {'T8-16', 'T8-16-Plus'};
group = HebiLookup.newGroupFromNames( familyName, moduleNames );
cmd = CommandStruct();

trajGen = HebiTrajectoryGenerator();



pos1 = [-pi/2 -pi/2];
pos2 = [3*pi/2 3*pi/2];
speed = -2;


posTargets = pos2;

% 
% while 1
%     fbk = group.getNextFeedback;
%     
%     gravcomp = 30*cos(fbk.position);
%     
%     cmd.effort = gravcomp;
%     cmd.position = pos1;
% %     cmd.velocity = [speed speed];
%     group.send(cmd);
%  
% end

fbk = group.getNextFeedback();
waypoints = [ fbk.position;
              posTargets(1,:) ];    % [rad]
timeToMove = 7.5;              % [sec]
time = [ 0 timeToMove ];     % [sec]

% Calculate initial trajectory to starting posiiton
trajectory = trajGen.newJointMove( waypoints, 'time', time );

% Initialize timer
t0 = fbk.time;
t = 0;

% Execute the motion to go to the first target
while t < trajectory.getDuration
    
    % Get feedback and update the timer
    fbk = group.getNextFeedback();
    t = fbk.time - t0;
    
    % Get new commands from the trajectory
    [pos,vel,acc] = trajectory.getState(t);
        
    gravcomp = 30*cos(fbk.position);
     
    % Fill in the CommandStruct and send commands to the arm
    cmd.position = pos;
    cmd.velocity = vel;
    cmd.effort = gravcomp;
    
    group.send(cmd);
end
