% -------------------------------------------------------------------------
% NOTE
% As seen in the basic grav comp example controlling only efforts to 
% compenstate for gravity will result in some amount of drift, depending on
% the accuracy of your mass models and torque sensing. 
%
% One way to mitigate this drift is by adding other layers of control, like
% a virtual spring that adds additional efforts to remain at the position
% where it was let go.
%
% ADDITIONAL NOTES
% Another way to get the same effect would be by commanding positions in
% strategy 4 as the position loop feeds into the torque loop.  In this case
% the Kp gains on the position loop on each module would correspond to the
% 'stiffness' parameter below.
% -------------------------------------------------------------------------

%% Setup
% Robot specific setup. Edit as needed.
[ group, kin, effortOffset, gravityVec ] = setupArm('4dof');

% Select the duration in seconds
demoDuration = 30;

% Velocity threshold below which robot is considered moving. Instead of
% trying to determine movement based on sensor data, it could also be a
% physical button on the robot that users would need to press before being
% able to move the robot.
velocityThreshold = 0.5;

% Stiffness (like Kp gain) of the virtual spring. i.e., how hard should  
% it try to keep the position. Can be different for each module (vector)
% and may need some tuning. A stiffness of all zeros effectively disables 
% the spring.
stiffness = 10 * ones(1,kin.getNumDoF());

%% Gravity compensated mode
fbk = group.getNextFeedback();
idlePos = fbk.position;
stiffness = stiffness .* ones(1,group.getNumModules()); % turn into vector

cmd = CommandStruct();
demoTimer = tic();
while toc(demoTimer) < demoDuration
    
    % Gather sensor data and always do grav comp
    fbk = group.getNextFeedback();
    
    cmd.effort = kin.getGravCompEfforts (fbk.position, gravityVec ) ...
        + effortOffset;
    
    % Find whether robot is actively moving
    isMoving = max(abs(fbk.velocity)) > velocityThreshold;
    if isMoving
        % Update idle position
        idlePos = fbk.position;
    else
        % Add efforts from virtual spring to maintain position
        driftError = idlePos - fbk.position;
        holdingEffort = driftError .* stiffness;
        cmd.effort = cmd.effort + holdingEffort;
    end
    
    % Send to robot
    group.send(cmd);
    
end