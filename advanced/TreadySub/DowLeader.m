% -------------------------------------------------------------------------
% NOTE
% Controlling only torques (or forces) will always exhibit some amount of 
% drift due to noise in the sensors and a non-perfect model of the robot. 
% This can be mitigated by adding an extra controller that can add torques
% to remain at a position when the robot is not actively beind held.
% -------------------------------------------------------------------------

%% Setup
% Robot specific setup. Edit as needed.
[group, group2, arm, kin, gravityVec] = setupArm();
arm.plugins = {HebiArmPlugins.DoubledJointMirror(2,group2)}
gains = HebiUtils.loadGains('7DofGains.XML');
group.send('gains',gains);
gains2 = HebiUtils.loadGains('mirrorGains.XML');
group2.send('gains',gains2);


% Select the duration in seconds
demoDuration =60;

% Setup Visualization
framesDisplay = FrameDisplay();

% Start Logging
group.startLog('dir', '/logs');

%% Gravity compensated mode
cmd = CommandStruct();

demoTimer = tic();

%gravity compensation warning
disp ('Commanded gravity-compensated zero torques to the arm');
disp ('Press ESC to Stop');

%initialize keyboard
kb = HebiKeyboard();
keys = read(kb);

while ~keys.ESC
% while (toc(demoTimer) < demoDuration) && ~keys.ESC
    
    keys = read(kb);
    
    arm.update();
    arm.send();
    
    % Get sensor data
    fbk = group.getNextFeedback();
    
    % Visualization
    positions = fbk.position; 
    frames = kin.getFK('output', positions);
    framesDisplay.setFrames(frames)
    drawnow;
    axis equal
end

%user feedback
if keys.ESC
    disp('Stopped')
else
    disp('Time expired')
end
% Stop Logging
log = group.stopLogFull();