%% Setup
% Robot specific setup. Edit as needed.
[group, kin, gravityVec] = setupArm_chevron();

cmd = CommandStruct();

% Select whether coordinate frames for static links should be drawn as well
showLinks = true;

% Length of the drawn axes
axisLength = 0.1; % [m]

% Drawable region (could auto-scale, but it's a bit disorienting)
drawLim = [-.5 .5]; % [m]

%% Passive Visualization
selected = kin.getBodyInfo().isDoF; 
if showLinks
   selected(:) = true; 
end

framesDisplay = FramesDisplay();

xlim(drawLim);
ylim(drawLim);
zlim(drawLim);

while true
    
    % Calculate kinematics
    fbk = group.getNextFeedback();
    frames = kin.getForwardKinematics('output', fbk.position);
    
    cmd.effort = kin.getGravCompEfforts( fbk.position, gravityVec );
    group.send(cmd);
    
    % Draw coordinate frames
    frames = frames(:,:,selected);
    framesDisplay.setFrames(frames);
    drawnow;
    
end