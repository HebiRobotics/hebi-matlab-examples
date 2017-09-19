% -------------------------------------------------------------------------
% !!!! WARNING !!!!
% Before running this code we recommend changing the allowed workspace
% in the 'getTargetCoordinates' function below and setting safety limits
% on the base/elbow/shoulder joints. You may increase the speed factor,
% but we recommend starting out slow.
% -------------------------------------------------------------------------

%% Setup
% Robot specific setup. Edit as needed.
[group, kin, gravityVec] = setupArm();

% Trajectory
trajGen = HebiTrajectoryGenerator(kin);
trajGen.setMinDuration(0.2); % Speed up 'small' movements (default is >1s)
trajGen.setSpeedFactor(0.5); % Slow down to a reasonable speed

% Select whether to add efforts (torques) to compensate for gravity and 
% dynamic effects. This can improve motions significantly and becomes
% especially important at high speeds. We recommend using control
% strategy 4 if enabled, and control strategy 3 if disabled.
enableEffortComp = true;

%% Continuously move to target 
fbk = group.getNextFeedbackFull();
cmd = CommandStruct();

% Move to current coordinates
targetXyz = getTargetCoordinates();
ikPosition = kin.getIK('xyz', targetXyz, ...
    'initial', zeros(1, group.getNumModules));

traj = trajGen.newJointMove([fbk.position; ikPosition]);

endVelocities = zeros(1, group.getNumModules);
endAccels = zeros(1, group.getNumModules);
lastXyz = [nan nan nan];

t0 = fbk.time;
while true
   
    % Gather feedback
    fbk = group.getNextFeedback(fbk); % reuse struct
    fbkPosition = fbk.position; % only access once
    
    % Get state of current trajectory
    t = min(fbk.time - t0, traj.getDuration()); % bound to max duration
    [pos,vel,accel] = traj.getState(t);
    cmd.position = pos;
    cmd.velocity = vel;
   
    if enableEffortComp
        dynamicsComp = kin.getDynamicCompEfforts(fbkPosition,pos,vel,accel);
        gravComp = kin.getGravCompEfforts(fbkPosition, gravityVec);
        cmd.effort = dynamicsComp + gravComp;
    end
    
    % Send current state to robot
    group.send(cmd);
    
    % Recompute trajectory if target has changed
    targetXyz = getTargetCoordinates();
    if any(lastXyz ~= targetXyz)
        lastXyz = targetXyz;
        
        % Find target joint positions using inverse kinematics
        ikPosition = kin.getIK('xyz', targetXyz, ...
            'TipAxis', [1 0 0], ... % keep output facing the same direction
            'initial', pos); % seed with current location
        
        % Start new trajectory at the current state
        t0 = fbk.time;
        traj = trajGen.newJointMove(...
            [pos; ikPosition], ...
            'Velocities', [vel; endVelocities],     ...
            'Accelerations', [accel; endAccels]);
        
    end
    
end

%% Get target coordinates (from mouse, but could also be from e.g. video)
function [xyz] = getTargetCoordinates()

% Select whole screen as valid area 
screenSize = get(0,'screensize');
screen_x = [0 screenSize(3)]; % (1) left - right, e.g., [0 1920]
screen_y = [0 screenSize(4)]; % (2) bottom - top, e.g., [0 1200]

% Read mouse input and bind within valid area [pixels]
loc = get(0, 'PointerLocation');
mouseX = min(max(loc(1), screen_x(1)), screen_x(2));
mouseY = min(max(loc(2), screen_y(1)), screen_y(2));

% Find relative coordinates in width and height
relativeLoc = [
    (mouseX - screen_x(1)) / diff(screen_x);
    (mouseY - screen_y(1)) / diff(screen_y);
];

% Set possible workspace range [m]
world_x = [+0.25 +0.50];
world_y = [+0.50 -0.25];
world_z = 0;

% Map input [pixels] to workspace [m]
x = relativeLoc(2) * diff(world_x) + world_x(1);
y = relativeLoc(1) * diff(world_y) + world_y(1);
z = world_z;
xyz = [x,y,z];

end
