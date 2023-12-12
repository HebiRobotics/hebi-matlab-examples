% -------------------------------------------------------------------------
% NOTE
% Controlling only torques (or forces) will always exhibit some amount of 
% drift due to noise in the sensors and a non-perfect model of the robot. 
% This can be mitigated by adding an extra controller that can add torques
% to remain at a position when the robot is not actively beind held.
% -------------------------------------------------------------------------

%% Setup
% Robot specific setup. Edit as needed.
[group, group2, arm2, kin, gravityVec] = setupArm();

gains = HebiUtils.loadGains('7DofGains.XML')
group.send('gains',gains);
gains2 = HebiUtils.loadGains('mirrorGains.XML')
group2.send('gains',gains2);




% Select the duration in seconds
demoDuration = 60;

% Setup Visualization
framesDisplay = FrameDisplay();

% Start Logging
 group.startLog;

%% Gravity compensated mode
cmd = CommandStruct();
cmd2 = CommandStruct();
demoTimer = tic();
while toc(demoTimer) < demoDuration
    
    % Gather sensor data
    fbk = group.getNextFeedback();
    
    % Calculate required torques to negate gravity at current position
    cmd.effort = kin.getGravCompEfforts( fbk.position, gravityVec );
    cmd.effort(2) = cmd.effort(2)/2
    cmd2.effort = -cmd.effort(2)
    
    % Get Feedback to create a Visualization
    positions = fbk.position; 
    frames = kin.getFK('output', positions);
    framesDisplay.setFrames(frames)
    drawnow;
    axis equal
    
    % Send to robot
      group.send(cmd);
      group2.send(cmd2);
    
end

% Stop Logging
log = group.stopLogFull();