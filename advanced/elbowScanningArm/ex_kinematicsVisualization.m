% This script does gravity compensation and shows the pose of the arm in
% real time.  Note that the base frame is set based on the estimated
% orientation of the IMU.  This means the base may drift in yaw over time.

%% Setup
% Robot specific setup. Edit as needed.
[group, kin, gravityVec] = setupArm_elbowScanner();

cmd = CommandStruct();

% Select whether coordinate frames for static links should be drawn as well
showLinks = true;

% Length of the drawn axes
axisLength = 0.1; % [m]

% 4x4 transform for setting the base frame of the kinematics
baseTransform = eye(4);

% Drawable region (could auto-scale, but it's a bit disorienting)
drawLim = [-.5 .5]; % [m]

%% Passive Visualization
selected = kin.getBodyInfo().isDoF; 
if showLinks
   selected(:) = true; 
end

framesDisp = FrameDisplay();

xlim(drawLim);
ylim(drawLim);
zlim(drawLim);

while true
    
    % Calculate kinematics
    fbk = group.getNextFeedbackFull();
    
    frames = kin.getForwardKinematics('output', fbk.position);
    
    % Orient the robot based on the estimated orientation from the first
    % module.  Commment this section of code out to visualize in the frame 
    % of the first module.
    baseQuaternion = [ fbk.orientationW(1), ...
                       fbk.orientationX(1), ...
                       fbk.orientationY(1), ...
                       fbk.orientationZ(1) ];
    baseRotMat = HebiUtils.quat2rotMat( baseQuaternion );
    baseTransform(1:3,1:3) = baseRotMat; 
    for i=1:size(frames,3)
        frames(:,:,i) = baseTransform * frames(:,:,i);
    end
    
    cmd.effort = kin.getGravCompEfforts( fbk.position, gravityVec );
    group.send(cmd);
    
    % Draw coordinate frames
    frames = frames(:,:,selected);
    framesDisp.setFrames(frames);
    drawnow;
    
end