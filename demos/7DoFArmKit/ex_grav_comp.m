% -------------------------------------------------------------------------
% NOTE
% Controlling only torques (or forces) will always exhibit some amount of 
% drift due to noise in the sensors and a non-perfect model of the robot. 
% This can be mitigated by adding an extra controller that can add torques
% to remain at a position when the robot is not actively beind held.
% -------------------------------------------------------------------------

%% Setup
% Robot specific setup. Edit as needed.
[group, kin, gravityVec] = setupArm();

gains = HebiUtils.loadGains('teachRepeatGains.XML');
group.send('gains',gains);

% Select the duration in seconds
demoDuration = 30;

% Setup Visualization
framesDisplay = FrameDisplay();

% Start Logging
% group.startLog;

%% Gravity compensated mode
cmd = CommandStruct();
demoTimer = tic();
while toc(demoTimer) < demoDuration
    
    % Gather sensor data
    fbk = group.getNextFeedback();
    
    % Calculate required torques to negate gravity at current position
    cmd.effort = kin.getGravCompEfforts( fbk.position, gravityVec );
    
    % Get Feedback to create a Visualization
    positions = fbk.position; 
    frames = kin.getFK('output', positions);
    framesDisplay.setFrames(frames)
    drawnow;
    axis equal
    
    % Send to robot
    group.send(cmd);
    
end

% Stop Logging
log = group.stopLogFull();