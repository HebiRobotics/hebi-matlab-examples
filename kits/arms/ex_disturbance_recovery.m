%% Setup
% Robot specific setup. Edit as needed.
[group, kin, gravityVec] = setupArm();

% Trajectory generator
trajGen = HebiTrajectoryGenerator(kin);
trajGen.setSpeedFactor(1);
trajGen.setMinDuration(0.5);

% Total runtime in seconds
runtime = 60; % [s]

% Detect disturbance
effErrThreshold = 4;
velThreshold = 0.5;

% Detect disturbance done
returnBelowVel = 0.3;

% Duration that the updated position is behind. Higher = more damping.
posDamping = 0.3; % [s]

%% Set home position (skip if already defined)
if ~exist('home', 'var')
    
    disp('Move robot to desired home position and press the ALT key');
    kb = HebiKeyboard();
    cmd = CommandStruct();
    while true
        
        % Do grav-comp while training waypoints
        fbk = group.getNextFeedback();
        cmd.effort = kin.getGravCompEfforts(fbk.position, gravityVec);
        group.send(cmd);
        
        % Exit on key press
        keys = read(kb);
        if keys.ALT == 1
            disp('Found home position')
            home = fbk.position;
            break;
        end

    end
    
end

%% Continuously try to return to home position
% Starting trajectory
endPosition = home;
endVelocities = home * 0;
endAccels = home * 0;
fbk = group.getNextFeedbackFull();
traj = trajGen.newJointMove([fbk.position; endPosition]);

% Starting state
cmd = CommandStruct();
disturbed = false;
t0 = fbk.time;
tic();
while toc() < runtime
    
    % Gather feedback
    fbk = group.getNextFeedback(fbk);
    fbkPosition = fbk.position;
    fbkVelocity = fbk.velocity;
    
    % Calculate dt based on feedback timestamp
    time = fbk.time;
    
    % Get state of current trajectory (cap time when reaching the end)
    t = min(traj.getDuration(), time - t0);
    [pos,vel,accel] = traj.getState(t);
    
    % Send commands to robot
    dynamicsComp = kin.getDynamicCompEfforts(fbkPosition,pos,vel,accel);
    gravComp = kin.getGravCompEfforts(fbkPosition,gravityVec);
    cmd.position = pos;
    cmd.velocity = vel;
    cmd.effort = dynamicsComp + gravComp;
    group.send(cmd);
    
    % Detect disturbance
    maxEffErr = max(abs(fbk.effort - fbk.effortCmd));
    maxVel = max(abs(fbkVelocity));
    if ~disturbed && maxEffErr > effErrThreshold && maxVel > velThreshold
        disp('Enter disturbance mode');
        disturbed = true;
    end
    
    % React to disturbance
    if disturbed
        
        % Update target to a position behind the current point
        endPosition = fbkPosition - fbkVelocity * posDamping;
        
        % Return 'home' when the robot has calmed down
        if maxVel < returnBelowVel
            disp('Exit disturbance mode');
            disturbed = false;
            endPosition = home;
        end
        
        % Replan from last point to stay continuous
        traj = trajGen.newJointMove(...
            [pos; endPosition], ...
            'Velocities', [vel; endVelocities],     ...
            'Accelerations', [accel; endAccels]);
        t0 = time;
        
    end
    
end

%% Comparison code for (default) virtual spring behavior (strategy 4)
% while true
%     fbk = group.getNextFeedback();
%     cmd.position = home;
%     cmd.velocity = endVelocities;
%     cmd.effort = kin.getGravCompEfforts(home, gravityVec);
%     group.send(cmd);
% end
