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
[ group, kin, effortOffset, gravityVec ] = setupArm('4dof');

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
[xyzTarget, tipTarget] = getTargetCoordinates(joy);
ikPosition = kin.getIK(...
    'xyz', xyzTarget, ...
    'TipAxis', tipTarget, ...
    'initial', zeros(1, group.getNumModules));

traj = trajGen.newJointMove([fbk.position; ikPosition]);

endVelocities = zeros(1, group.getNumModules);
endAccels = zeros(1, group.getNumModules);

xyzLast = [nan nan nan];
tipLast = [nan nan nan]';

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
    [xyzTarget, tipTarget] = getTargetCoordinates(joy);
    if any(xyzLast ~= xyzTarget) || any(tipLast ~= tipTarget)
        xyzLast = xyzTarget;
        tipLast = tipTarget;
        
        % Find target joint positions using inverse kinematics
        if kin.getNumDoF >= 5
            ikPosition = kin.getIK( 'xyz', xyzTarget, ...
                                    'TipAxis', tipTarget, ...
                                    'initial', pos); % seed with current location
        else
            ikPosition = kin.getIK( 'xyz', xyzTarget, ...
                                    'initial', pos); % seed with current location
        end
        
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
HebiUtils.plotLogs(hebilog, 'position');
HebiUtils.plotLogs(hebilog, 'velocity');
HebiUtils.plotLogs(hebilog, 'effort');

%% Get target coordinates and tip axis (from joystick)
function [xyz, tipTarget] = getTargetCoordinates(joy)

% Set possible workspace range [m]
world_x = [+0.25 +0.60];
world_y = [-0.25 +0.25];
world_z = [-0.10 +0.10];
angle_y = [-pi/2 +pi/2];

% Find relative coordinates in x and y [0,1]. (Note: axes are [-1,+1]
[axes, ~, ~] = read(joy);
relativeLoc = (-axes + 1) / 2;

% Map input [0,1] to workspace [m] (linear k * x + d)
x = relativeLoc(2) * diff(world_x) + world_x(1);
y = relativeLoc(1) * diff(world_y) + world_y(1);
z = relativeLoc(3) * diff(world_z) + world_z(1);
xyz = [x,y,z];

% Map input [0,1] to y rotation [rad]
yAngle = relativeLoc(5) * diff(angle_y) + angle_y(1);
tipTarget = [0 0 1]';

% Rotate vector using rotation matrix 
s = sin(yAngle); 
c = cos(yAngle);
Ry = [
  c 0 s
  0 1 0
  -s 0 c];
tipTarget = Ry * tipTarget;

end
