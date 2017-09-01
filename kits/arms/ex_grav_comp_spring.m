% -------------------------------------------------------------------------
% NOTE
% As seen in the basic grav comp example controlling only efforts will
% result in some amount of drift. One way to mitigate this is by adding
% a virtual spring that adds additional efforts to remain at the position
% where it was let go.
%
% ADVANCED NOTE
% Another way to get the same effect would be by commanding positions in
% strategy 4 as the position loop feeds into the torque loop.
% -------------------------------------------------------------------------

%% Setup
% Robot specific setup. Edit as needed.
[group, kin, gravityVec] = setupArm();

% Select the duration in seconds
duration = 60;

% Velocity threshold below which robot is considered moving. Instead of
% trying to determine movement based on sensor data, it could also be a
% physical button on the robot that users would need to press before being
% able to move the robot.
velocityThreshold = 0.2;

% Stiffness (like Kp gain) of the virtual spring, i.e., how hard should  
% it try to keep the position. Could be different for each module (vector)
% and may need some tuning. A stiffness of zero disables it.
stiffness = 5;

%% Gravity compensated mode
fbk = group.getNextFeedback();
idlePos = fbk.position;
stiffness = stiffness .* ones(1,group.getNumModules()); % turn into vector

cmd = CommandStruct();
t0 = tic();
while toc(t0) < duration
    
    % Gather sensor data and always do grav comp
    fbk = group.getNextFeedback(fbk);
    fbkPosition = fbk.position;
    cmd.effort = kin.getGravCompEfforts(fbkPosition, gravityVec);
    
    % Find whether robot is actively moving
    isMoving = max(abs(fbk.velocity)) > velocityThreshold;
    if isMoving
        % Update idle position
        idlePos = fbkPosition;
    else
        % Add efforts from virtual spring to maintain position
        driftError = idlePos - fbkPosition;
        holdingEffort = driftError .* stiffness;
        cmd.effort = cmd.effort + holdingEffort;
    end
    
    % Send to robot
    group.send(cmd);
    
end