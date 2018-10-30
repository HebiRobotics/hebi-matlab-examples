% -------------------------------------------------------------------------
% NOTE
% Controlling only torques (or forces) will always exhibit some amount of 
% drift due to noise in the sensors and a non-perfect model of the robot. 
% This can be mitigated by adding an extra controller that can add torques
% to remain at a position when the robot is not actively beind held.
% -------------------------------------------------------------------------

%% Setup
% Robot specific setup. Edit as needed.
[ group, kin, effortOffset, gravityVec ] = setupArm('4dof');

% Select the duration in seconds
demoDuration = 30;

%% Gravity compensated mode
cmd = CommandStruct();
demoTimer = tic();
while toc(demoTimer) < demoDuration
    
    % Gather sensor data
    fbk = group.getNextFeedback();
    
    % Calculate required torques to negate gravity at current position
    cmd.effort = kin.getGravCompEfforts( fbk.position, gravityVec ) ...
        + effortOffset;
    
    % Send to robot
    group.send(cmd);
    
end
