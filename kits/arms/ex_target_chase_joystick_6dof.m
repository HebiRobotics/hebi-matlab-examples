% -------------------------------------------------------------------------
% This demo is the same as ex_target_chase.m, with the difference being
% that the target is based on joystick input rather than the mouse
% location, and that it includes a variable tip axis.
%
% !!!! WARNING !!!!
% Before running this code we recommend changing the allowed workspace
% in the 'getTargetCoordinates' function below and setting safety limits
% on the base/elbow/shoulder joints. You may increase the speed factor,
% but we recommend starting out slow.
% -------------------------------------------------------------------------

%% Setup
% Robot specific setup. Edit setupArm.m as needed for your configuration.
[ group, kin, effortOffset, gravityVec ] = setupArm('6dof');

% Trajectory
trajGen = HebiTrajectoryGenerator(kin);
trajGen.setMinDuration(0.2); % Speed up 'small' movements (default is >1s)
trajGen.setSpeedFactor(1); % Speed multiplier (1 = full speed, 0.5 = half)

% Select whether to add efforts (torques) to compensate for gravity and 
% dynamic effects. This can improve motions significantly and becomes
% especially important at high speeds. We recommend using control
% strategy 4 if enabled, and control strategy 3 if disabled.
enableEffortComp = true;

% Connect to joystick
joy = HebiJoystick(1);

%% Continuously move to target 
fbk = group.getNextFeedbackFull();
cmd = CommandStruct();

% Start background logging
group.startLog();

% Move to current coordinates
[xyzTarget, so3] = getTargetCoordinates(joy);
ikPosition = kin.getIK(...
    'xyz', xyzTarget, ...
    'so3', so3, ...
    'initial', [0 1 2.5 -1 1 1]);

traj = trajGen.newJointMove([fbk.position; ikPosition]);

endVelocities = zeros(1, group.getNumModules);
endAccels = zeros(1, group.getNumModules);

xyzLast = [nan nan nan];
so3Last = [nan nan nan]';

t0 = fbk.time;

maxDemoTime = 30; % sec
demoTimer = tic;

while toc(demoTimer) < maxDemoTime
   
    % Gather feedback
    fbk = group.getNextFeedback();

    t = min(fbk.time - t0, traj.getDuration()); % bound to max duration
    
    % Get state of current trajectory
    [pos,vel,accel] = traj.getState(t);
    cmd.position = pos;
    cmd.velocity = vel;
   
    if enableEffortComp
        dynamicsComp = kin.getDynamicCompEfforts( ...
                                    fbk.position, pos, vel, accel );
        gravComp = kin.getGravCompEfforts( fbk.position, gravityVec );
        cmd.effort = dynamicsComp + gravComp + effortOffset;
    end
    
    % Send current state to robot
    group.send(cmd);
    
    % Recompute trajectory if target has changed
    [xyzTarget, so3] = getTargetCoordinates(joy);
    if any(xyzLast ~= xyzTarget) || any(any(so3Last ~= so3))
        xyzLast = xyzTarget;
        so3Last = so3;
        
        % Find target using inverse kinematics
        ikPosition = kin.getIK( 'xyz', xyzTarget, ...
            'so3', so3, ...
            'initial', pos); % seed with current location
        
        % Start new trajectory at the current state
        t0 = fbk.time;
        traj = trajGen.newJointMove( [pos; ikPosition], ...
                        'Velocities', [vel; endVelocities], ...
                        'Accelerations', [accel; endAccels]);  
    end
    
end

% Stop background logging
hLog = group.stopLogFull();

% Plot the commands and feedback
HebiUtils.plotLogs(hebilog, 'position', 'figNum', 101);
HebiUtils.plotLogs(hebilog, 'velocity', 'figNum', 102);
HebiUtils.plotLogs(hebilog, 'effort', 'figNum', 103);

%% Get target coordinates and tip axis (from joystick)
function [xyz, so3] = getTargetCoordinates(joy)

% Set possible workspace range [m]
world_x = [+0.15 +0.65];
world_y = [-0.25 +0.25];
world_z = [-0.20 +0.10];
angle_x = [+pi/2 -pi/2];
angle_y = [+3*pi/2 +pi/2];

% Linear mapping function (k * x + d)
mapLinear = @(range, relativeLoc) relativeLoc * diff(range) + range(1);

% Find relative coordinates in x and y [0,1]. (Note: axes are [-1,+1]
[axes, ~, ~] = read(joy);
relativeLoc = (-axes + 1) / 2;

% Map input [0,1] to workspace [m] (linear k * x + d)
x = mapLinear(world_x, relativeLoc(2));
y = mapLinear(world_y, relativeLoc(1));
z = mapLinear(world_z, relativeLoc(3));
xyz = [x,y,z];

% Map input [0,1] to x/y rotation [rad]
xAngle = mapLinear(angle_x, relativeLoc(4));
yAngle = mapLinear(angle_y, relativeLoc(5));
so3 = R_y(yAngle) * R_x(xAngle);

end
