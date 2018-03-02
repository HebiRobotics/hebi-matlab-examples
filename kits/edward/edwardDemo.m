% Code for the edward/omni base demo
%
% Features:      joystick input to control motions of 3 omni wheel base
%                also controls spine and arms for edward robot kit
%
% Requirements:  MATLAB 2013b or higher
%
% Author:        Andrew Willig
% Date:          14 November, 2017
%

% Copyright 2017-2018 HEBI Robotics

function edwardDemo()
%% Setup

% Optional step to limit the lookup to a set of interfaces or modules
% HebiLookup.setLookupAddresses('10.10.10.255');

% Setup Group
moduleNames = {
    'wheel1', ...   % right front omni wheel
    'wheel2', ...   % left front omni wheel
    'wheel3', ...   % back middle omni wheel
    'ankle', 'knee', ...    % spine actuators
    'baseLeft', 'shoulderLeft', 'elbowLeft', 'wristLeft', ...
    'baseRight', 'shoulderRight', 'elbowRight', 'wristRight' };

wheelDOFs = 1:3;
spineDOFs = 4:5;
armDOFs{1} = 6:9;
armDOFs{2} = 10:13;
numDOFs = length(moduleNames);

fprintf('Searching for modules...\n');
createGroup = @()HebiLookup.newGroupFromNames('Edward', moduleNames);
robotGroup = DemoUtils.retryOnError(createGroup);
fprintf('Found.\n');

% Make sure modules are in application mode
DemoUtils.bootIntoApplication(robotGroup);

% Load pre-set gains
localDir = fileparts(mfilename('fullpath'));
gains = HebiUtils.loadGains(fullfile(localDir, 'edwardGains'));
DemoUtils.sendGains(robotGroup, gains);

% Setup Joystick
fprintf('Searching for joystick...\n');
createJoystick = @() HebiJoystick(1);
joy = DemoUtils.retryOnError(createJoystick);
joy = SonyPS4Gamepad(joy);
joy.axisLowpass = 0.2; % (filter noisy user inputs)
fprintf('Found.\n');

% Workaround for bug that makes the triggers not work correctly until they
% are both pushed down fully for the first time. Also checks for a bug where
% all of the axes are initialized to -1 in linux.
while ~joy.hasInitialConditions()
    robotGroup.send('led','b');
    pause(0.1);
    robotGroup.send('led','m');
    pause(0.1)
end

%% Main
while true
    
    %% Start/Stop Edward Demo
    fprintf('Paused. Click left stick to start, share to quit matlab...\n');  

    joyState = joy.read();
    while joyState.BUTTON_LEFT_STICK_CLICK == 0
        
        joyState = joy.read();
        if joyState.BUTTON_SHARE
            robotGroup.send('led',[]);
            quit force
        end
        pause(0.1);
        robotGroup.send('led','b');
        pause(0.1);
        robotGroup.send('led','g');
  
    end
    
    fprintf('Running. Click left stick to stop...\n');
    
    % TODO: needs reset of joystick axes?
    
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Setup / Initialization                                              %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Create models
    [param, kin, trajGen] = setupEdward();
    
    % Get initial feedback
    onError = @()disp('Could not get feedback');
    fbk = DemoUtils.retryOnError(@()robotGroup.getNextFeedbackFull(), onError);
    timeNow = fbk.time;
    timeLast = timeNow;
    
    logging = false;  % Flag to turn logging on and off.  If logging is on you
                      % you can view a bunch of debug plots after quitting.
    if logging
        timeHist = nan(0,1);
        RPYHist = nan(0,3);
        robotGroup.startLog();
    end
    
    gripVel = zeros(3,param.numArms);
    wristVel = 0;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % INITIALIZE HOME POSITIONS %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Spine
    ankleAngInit = atan2(param.upperSpineLink,param.lowerSpineLink);
    ankleOffset = deg2rad(5);
    kneeAngle = deg2rad(90);
    ankleAngle = pi/2 + ankleAngInit + ankleOffset;
    spinePosHome = [ankleAngle  kneeAngle];
    
    % Base frames for the arms
    spineTipFK = kin.SPINE.getFK('endeffector', spinePosHome);
    armBaseFrame = eye(4);
    armBaseFrame(1:3,1:3) = spineTipFK(1:3,1:3);
    
    % Arms
    armPosHome = zeros(4,param.numArms);
    gripPos = zeros(3,param.numArms);
    for arm = 1:param.numArms
        
        % Set base frame / coordinate system
        kin.ARMS{arm}.setBaseFrame(armBaseFrame);
        
        % Joint positions
        armPosHome(:,arm) = param.armDir{arm} * deg2rad([90 90 -90 0]);
        
        % XYZ gripper positions
        T_endEffector = kin.ARMS{arm}.getFK('endeffector',armPosHome(:,arm));
        gripPos(:,arm) = T_endEffector(1:3,4);
        
    end
    
    % Initialize the gripper position to the average of both arms. The
    % arms are mirrored, so the x axis on one arm is negative.
    gripMirror = ones(size(gripPos));
    gripMirror(1,2) = -1;
    gripPos = mean(gripPos .* gripMirror, 2) .* gripMirror;
        
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Soft Startup (moves towards the desired starting position)          %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    startupTime = 3.0;
    time = [ 0 startupTime ];
    minRampTime = .5;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    % Initialize pose filter %
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    poseFilter = HebiPoseFilter();
    poseFilter.setMaxAccelWeight( .01 );
    poseFilter.setMaxAccelNormDev( .3 );
    filterTime = timeNow;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Create Homing Trajectories %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Omni Base trajectory
    % Note that this does not get used during warmup and that it will be 
    % replanned on the first iteration of the main loop
    chassisTraj = trajGen.CHASSIS.newJointMove( zeros(2,3), 'Time', time );
    chassisTrajStartTime = timeNow;
    
    % Spine Trajectory
    positions = [ fbk.position(spineDOFs); spinePosHome];
    spineTraj = trajGen.SPINE.newJointMove( positions, 'Time', time );
    spineTrajStartTime = timeNow;
    
    % Arm Trajectories
    armTraj = cell(1,param.numArms);
    armTrajStartTime = cell(1,param.numArms);
    for i = 1:param.numArms
        positions = [ fbk.position(armDOFs{i}); armPosHome(:,i)' ];
        armTraj{i} = trajGen.ARMS{i}.newJointMove( positions, 'Time', time );
        armTrajStartTime{i} = timeNow;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Command Homing Trajectories %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    cmd = CommandStruct();
    cmd.position = nan(1,numDOFs);
    cmd.velocity = nan(1,numDOFs);
    cmd.effort = nan(1,numDOFs);
    
    t0 = tic;
    t = toc(t0);
    while t < startupTime
        t = toc(t0);
        
        %%%%%%%%%%%%%%%%%%%
        % Gather Feedback %
        %%%%%%%%%%%%%%%%%%%
        try
            fbk = robotGroup.getNextFeedback( fbk );
        catch
            disp('Could not get feedback!');
            break;
        end
        timeNow = fbk.time;
        dt = mean(timeNow - timeLast);
        dt = min(dt,.02);
        timeLast = timeNow;
        filterTime = filterTime + dt;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Update filtered pose estimate %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        fbkAccel = [fbk.accelX; fbk.accelY; fbk.accelZ];
        fbkGyro = [fbk.gyroX; fbk.gyroY; fbk.gyroZ];

        % Get IMUs from the wheels into the same frame
        poseAccel = fbkAccel(:,wheelDOFs);
        poseGyro = fbkGyro(:,wheelDOFs);
        for j=1:length(wheelDOFs)
            poseAccel(:,j) = param.wheelBaseFrames(1:3,1:3,j) * poseAccel(:,j);
            poseGyro(:,j) = param.wheelBaseFrames(1:3,1:3,j) * poseGyro(:,j);
        end
        
        % Use average to update filter
        poseAccelMean = mean(poseAccel,2);
        poseGyroMean = mean(poseGyro,2);
        poseFilter.update( poseAccelMean, poseGyroMean, filterTime );
        T_pose = poseFilter.getPose();
        gravityVec = -T_pose(3,1:3);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Command Trajectory State %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Spine
        [spineCmd.pos, spineCmd.vel, ~] = spineTraj.getState(t);
        cmd.position(spineDOFs) = spineCmd.pos;
        cmd.velocity(spineDOFs) = spineCmd.vel;
        
        % Calculate base frame for the arms
        spineTipFK = kin.SPINE.getFK('endeffector', spineCmd.pos);
        armBaseFrame = eye(4);
        armBaseFrame(1:3,1:3) = spineTipFK(1:3,1:3);
        
        % Arms
        armCmd = cell(1,2);
        for i=1:param.numArms
            
            [armCmd{i}.pos, armCmd{i}.vel, ~] = armTraj{i}.getState(t);
            cmd.position(armDOFs{i}) = armCmd{i}.pos;
            cmd.velocity(armDOFs{i}) = armCmd{i}.vel;
            
            % Gravity Compensation Torques
            kin.ARMS{i}.setBaseFrame(armBaseFrame);
            cmd.effort(armDOFs{i}) = kin.ARMS{i}.getGravCompEfforts( ...
                fbk.position(armDOFs{i}), gravityVec );
            
        end
        
        % Initially ramp up efforts to avoid jumps
        softStart = min(t/startupTime,1);
        cmd.effort = softStart * cmd.effort;
        
        % Send to robot
        robotGroup.send(cmd);
        
    end
    
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Main Control Loop                                                   %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    warmupTimer = tic;
    warmupDuration = 1;  % sec
    
    wheelCmd.pos = fbk.position(wheelDOFs)';
    spinePos = spineCmd.pos;
    armPos = armPosHome;
    armVel = zeros(size(armPos));
    
    while true
        
        % Slowly increase efforts for impedance controller
        softStart = min(toc(warmupTimer)/warmupDuration,1);
        
        %%%%%%%%%%%%%%%%%%%
        % Gather Feedback %
        %%%%%%%%%%%%%%%%%%%
        try
            fbk = robotGroup.getNextFeedback( fbk );
        catch
            disp('Could not get feedback!');
            break;
        end
        timeNow = fbk.time;
        dt = mean(timeNow - timeLast);
        dt = min(dt,.01);
        timeLast = fbk.time;
        filterTime = filterTime + dt;

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Update filtered pose estimate %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        fbkAccel = [fbk.accelX; fbk.accelY; fbk.accelZ];
        fbkGyro = [fbk.gyroX; fbk.gyroY; fbk.gyroZ];

        % Get IMUs from the wheels into the same frame
        poseAccel = fbkAccel(:,wheelDOFs);
        poseGyro = fbkGyro(:,wheelDOFs);
        for j=1:length(wheelDOFs)
            poseAccel(:,j) = param.wheelBaseFrames(1:3,1:3,j) * poseAccel(:,j);
            poseGyro(:,j) = param.wheelBaseFrames(1:3,1:3,j) * poseGyro(:,j);
        end
        
        % Use average to update filter
        poseAccelMean = mean(poseAccel,2);
        poseGyroMean = mean(poseGyro,2);
        poseFilter.update( poseAccelMean, poseGyroMean, filterTime );
        T_pose = poseFilter.getPose();
        gravityVec = -T_pose(3,1:3);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Read/Map Joystick Inputs %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Read joystick state
        joyState = read(joy);
        if joyState.BUTTON_LEFT_STICK_CLICK
            break;
        end
        
        % Spine Height
        spineVelRatio = [0.5 1]; % [ankle moves at half speed of knee]
        if joyState.BUTTON_OPTIONS
            % Use button 10 (options) for a soft shutdown procedure
            spineVel = -0.25 * spineVelRatio;
            
            % Lower robot until kneeAngle threshold before exiting
            if(spinePos(2) < deg2rad(45))
                break;
            end
        else
            % Normal Stance Height control
            triggerAxis = joyState.AXIS_LEFT_TRIGGER - joyState.AXIS_RIGHT_TRIGGER;
            spineVel = 0.15 * triggerAxis * spineVelRatio;
        end
        
        % Arm X-Axis (2nd arm is mirrored)
        gripVel(1,:) = +0.2 * joyState.AXIS_LEFT_STICK_X .* [1 -1];
        
        % Arm Y-Axis
        gripVel(2,:) = -0.2 * joyState.AXIS_LEFT_STICK_Y;
        
        % Arm Z-Axis
        switch(joyState.POV)
            case 0 % forward
                desiredGripVel = +0.2;
            case 180 % backward
                desiredGripVel = -0.2;
            otherwise
                desiredGripVel = 0;
        end
        gripVel(3,:) = joy.axisLowpass * desiredGripVel + ...
            (1-joy.axisLowpass) * gripVel(3,:);
        
        % Wrist Rotation
        switch(joyState.POV)
            case 90 % right
                desiredWristVel = +2.5;
            case 270 % left
                desiredWristVel = -2.5;
            otherwise
                desiredWristVel = 0;
        end
        wristVel = joy.axisLowpass * desiredWristVel + ...
            (1-joy.axisLowpass) * wristVel;
        
        % Chassis Linear Velocity
        xVel = joyState.AXIS_RIGHT_STICK_X * param.maxLinSpeed; % m/s
        yVel = -joyState.AXIS_RIGHT_STICK_Y * param.maxLinSpeed; % m/s
        
        % Chassis Rotational Velocity
        rotAxis = joyState.BUTTON_RIGHT_TRIGGER - joyState.BUTTON_LEFT_TRIGGER;
        rotVel = rotAxis * param.maxRotSpeed; % rad/s
        chassisDesired.vel = [xVel; yVel; rotVel]';
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Evaluate Trajectory State %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Chassis (linear velocity)
        t = timeNow - chassisTrajStartTime;
        [chassisCmd.vel, chassisCmd.accel, chassisCmd.jerk] = chassisTraj.getState(t);
        
        % Chassis (convert linear to joint velocities)
        % Note that the maximum desired linear velocity is already limited, 
        % so we don't need to separately check whether the joint velocities 
        % along the trajectory are within achievable limits.
        wheelCmd.vel = param.chassisToWheelVelocities * chassisCmd.vel';
        wheelCmd.pos = wheelCmd.pos + wheelCmd.vel * dt;
        
        % Spine
        t = timeNow - spineTrajStartTime;
        [spineCmd.pos, spineCmd.vel, spineCmd.accel] = spineTraj.getState(t);
        
        % Arms
        armCmd = cell(1,param.numArms);
        for i = 1:param.numArms
            t = timeNow - armTrajStartTime{i};
            [armCmd{i}.pos, armCmd{i}.vel, armCmd{i}.accel] = armTraj{i}.getState(t);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Update Forward Kinematics %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % Spine
        spineTipCmdFK = kin.SPINE.getFK('endeffector', spineCmd.pos);
        
        % Note: The arm motion should be independent of the spine movement,
        % i.e., if the spine goes up, the arms should go up as well. By
        % removing the translational part from the base frame, we can avoid
        % having to compensate for this offset later on.
        armBaseFrame = eye(4);
        armBaseFrame(1:3,1:3) = spineTipCmdFK(1:3,1:3);
        
        % Arms
        armTipFK = cell(1,param.numArms);
        J_armFbk = cell(1,param.numArms);
        for i=1:param.numArms
            fbkPos = fbk.position(armDOFs{i});
            kin.ARMS{i}.setBaseFrame(armBaseFrame);
            armTipFK{i} = kin.ARMS{i}.getFK('endeffector', fbkPos);
            J_armFbk{i} = kin.ARMS{i}.getJacobianEndEffector(fbkPos );
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Constrain the Spine Motion %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % The knee/ankle joints typically move in a 2:1 ratio. Because both
        % links are the same length, this moves the tip (T-junction point) 
        % up and down in a straight line until the ankle hits a hard stop.
        % After that, only the knee moves, which results in the tip bending
        % forward. The motion needs to stop before the platform would fall 
        % over.
        %
        % Thus, the movement at the tip looks somewhat like the following:
        %
        %                 |
        %                 |
        %               __|
        %              /
        
        % Physical workspace limits [ankle knee]
        spinePosMax = [deg2rad(150) deg2rad(120)];
        spinePosMin = [deg2rad(115) deg2rad(35)];
        
        % Compensate for chassis movement on e.g. an incline via the ankle
        inclineVel = fbk.gyroZ(spineDOFs(1));
        spineVel(1) = spineVel(1) - inclineVel;
        spineDirection = sign(spineVel(2));
        
        % Integrate the desired velocities as if we had an unconstrained
        % system. This preserves the ratio when moving in and out of
        % partially constrained parts of the workspace. The integration
        % stops when all joints are outside their bounds in order to avoid
        % reaction delays due to windup.
        if ...
                (all(spinePos > spinePosMax) && spineDirection > 0) || ...
                (all(spinePos < spinePosMin) && spineDirection < 0)
            % Fully outside bounds. Still keep track of incline velocity,
            % so it won't accidentally miss updates.
            spinePos(1) = spinePos(1) - inclineVel * dt;
        else
            % Fully or partially inside bounds
            spinePos = spinePos + spineVel * dt;
        end
        
        % Limit the desired velocities to what is physically achievable
        spineDesired.pos = spinePos;
        spineDesired.vel = spineVel;
        
        above = spineDesired.pos > spinePosMax;
        below = spineDesired.pos < spinePosMin;
        
        if any(above)
            spineDesired.pos(above) = spinePosMax(above);
            spineDesired.vel(above) = 0;
        elseif any(below)
            spineDesired.pos(below) = spinePosMin(below);
            spineDesired.vel(below) = 0;
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Constrain the Arm End Effector Motion %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Keep users from getting into the singularity by keeping 
        % (one) elbow from straightening. Note that positivity is not
        % enforced and that the robot can still be switched between elbow
        % up/down configurations with manual intervention
        
        % Update linear XYZ gripper position
        newGripPos = gripPos + gripVel*dt;
        newArmPos = armPos;
        newArmVel = armVel;
        
        % Limit Gripper Width
        gripWidthLim = -.010; % [m]
        if newGripPos(1,1) > gripWidthLim
            newGripPos(1,:) = [gripWidthLim, -gripWidthLim];
        end
        
        % Convert linear translation to joint space
        ikValid = true;
        for i=1:param.numArms
            
            % Get desired joint positions using IK. Note that we are using
            % a 3 dof constraint on a 4 dof arm, so the wrist is
            % unconstrained and have an arbitrary value.
            ikPos = kin.ARMS{i}.getIK( ...
                'xyz', newGripPos(:,i), ...
                'initial', fbk.position(armDOFs{i}) );
            
            % Check that IK found a valid solution
            ikResult = kin.ARMS{i}.getFK('EndEffector', ikPos);
            ikError = abs(newGripPos(:,i) - ikResult(1:3,4));
            ikValid = ikValid && all(ikError < 0.01); % xyz err in [m]
            
            % Check spherical coordinates to make sure that the arm
            % isn't close to the singularity. (use shoulder module as
            % origin)
            armOutFrames = kin.ARMS{i}.getFK('out',ikPos);
            armBaseOutputFrameXYZ = armOutFrames(1:3,4,4);
            newGripPosSpherical = newGripPos(:,i) - armBaseOutputFrameXYZ;

            armRhoLim = 0.625; % m (max reach)
            newGripPosToRho = sqrt( sum( newGripPosSpherical .^2 ) );
            ikValid = ikValid && newGripPosToRho < armRhoLim;
            
            % Get desired joint velocities using Jacobian inverse
            newArmPos(:,i) = ikPos;
            newArmVel(:,i) = J_armFbk{i}(1:3,:) \ gripVel(:,i);
            
        end

        % Update first 3 joints
        if ikValid
            gripPos = newGripPos;
            armPos(1:3,:) = newArmPos(1:3,:);
            armVel(1:3,:) = newArmVel(1:3,:);
        else
            % Stop at previous arm positions
            armVel(1:3,:) = 0;
        end
        
        % Handle the wrist separately. Compensate for motions of shoulder
        % and elbow to keep wrist stable.
        armVel(4,:) = armVel(2,:) + armVel(3,:) + wristVel * [1, -1];
        armPos(4,:) = armPos(4,:) + armVel(4,:) * dt;
        
        % Convert to cell struct
        armDesired = cell(1,param.numArms);
        for i = 1:param.numArms
            armDesired{i}.pos = armPos(:,i)';
            armDesired{i}.vel = armVel(:,i)';
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % ADD EXTRA EFFORTS TO THE SPINE %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % For position and velocity control we can treat each limb as a
        % fully independent system, but adding efforts requires a coupled
        % approach, i.e., the gravitational effects on the spine depend on
        % the configuration of the arms.
        % The easiest way to do this in our current APIs is to treat the
        % spine as an RR manipulator with the combination of both arms 
        % as a single payload.
        
        % Calculate the center of mass for each arm
        armCoMs = zeros(3,param.numArms);
        armMasses = zeros(1,param.numArms);
        for i = 1:param.numArms
            
           % Get coordinates in end effector (spine tip) frame.
           % Alternatively we could just transform the result.
           kin.ARMS{i}.setBaseFrame(eye(4));
           
           % Calculate center of mass by doing a weighted average of all
           % individual bodies
           frames = kin.ARMS{i}.getFK('com', fbk.position(armDOFs{i}));
           bodyCoMs = squeeze(frames(1:3,4,:));
           masses = kin.ARMS{i}.getBodyMasses();
           armMasses(i) = sum(masses);
           armCoMs(:,i) = sum(bodyCoMs .* repmat(masses',3,1), 2) / armMasses(i);
            
           % Reset frame
           kin.ARMS{i}.setBaseFrame(armBaseFrame);
           
        end
        
        % Combine both arms into a single payload
        upperMass = sum(armMasses);
        upperCoM = sum(armCoMs .* repmat(armMasses,3,1), 2) / upperMass;
        kin.SPINE.setPayload(upperMass, 'com', upperCoM);
        
        % Gravity Compensation Efforts
        spineGravCompEffort = kin.SPINE.getGravCompEfforts( ...
            fbk.position(spineDOFs), gravityVec );
        
        % Dynamics Compensation Efforts
        spineDynamicsCompEffort = kin.SPINE.getDynamicCompEfforts( ...
            fbk.position(spineDOFs), ...
            spineCmd.pos, ...
            spineCmd.vel, ...
            spineCmd.accel );
        
        spineCmd.effort = spineGravCompEffort + spineDynamicsCompEffort;
        spineCmd.effort = softStart * spineCmd.effort;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % ADD EXTRA EFFORTS TO THE ARMS %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Arms are controlled in a more traditional way with position /
        % velocity and efforts like with fixed base manipulation.
        
        % Impedance Control Params
        damperGains = [1; 1; 1; .0; .0; .0;]; % N or Nm / m/s
        springGains = [150; 150; 150; 0; 0; 0];  % N/m or Nm/rad
        
        for i = 1:param.numArms
            
            % Impedance Control Torques
            xyzError = newGripPos(:,i) - armTipFK{i}(1:3,4);
            posError = [xyzError; zeros(3,1)];
            
            velError = J_armFbk{i} * ( armDesired{i}.vel - ...
                fbk.velocity(armDOFs{i}) )';
            
            impedanceTorque = J_armFbk{i}' * ...
                (springGains .* posError + ...
                damperGains .* velError);
            
            % Gravity Compensation Efforts
            armGravCompEffort = kin.ARMS{i}.getGravCompEfforts( ...
                fbk.position(armDOFs{i}), gravityVec );
            
            % Dynamics Compensation Efforts
            armDynamicsCompEffort = kin.ARMS{i}.getDynamicCompEfforts( ...
                fbk.position(armDOFs{i}), ...
                armCmd{i}.pos, ...
                armCmd{i}.vel, ...
                armCmd{i}.accel );
            
            % Add a soft start to the impedance controller
            armCmd{i}.effort = softStart*impedanceTorque' ...
                + armGravCompEffort ...
                + armDynamicsCompEffort;
            
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % SEND COMMANDS TO THE ROBOT %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Reset the Command Struct
        cmd.position = nan(1,numDOFs);
        cmd.velocity = nan(1,numDOFs);
        cmd.effort = nan(1,numDOFs);
        
        % Add commands for the wheels
        cmd.position(wheelDOFs) = wheelCmd.pos;
        cmd.velocity(wheelDOFs) = wheelCmd.vel;
        
        % Add commands for the spine
        cmd.position(spineDOFs) = spineCmd.pos;
        cmd.velocity(spineDOFs) = spineCmd.vel;
%         cmd.effort(spineDOFs) = spineCmd.effort; % optional
        
        % Add commands for the arms
        for i = 1:param.numArms
            cmd.position(armDOFs{i}) = armCmd{i}.pos;
            cmd.velocity(armDOFs{i}) = armCmd{i}.vel;
            cmd.effort(armDOFs{i}) = armCmd{i}.effort;
        end
        
        % Send to robot
        robotGroup.send(cmd);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % REPLAN TRAJECTORIES TOWARDS UPDATED GOALS %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        rampTime = minRampTime;
        time = [ 0 rampTime ];
        
        % Chassis (linear velocity)
        %
        % Note that rather than lowpassing the user input, we calculate
        % minimum jerk trajectories to smoothly transition from the current 
        % to the desired target velocity (i.e. cruising speed).
        %
        % TODO: should we use the built-in time heuristic to limit the
        % maximum accelerations? e.g. setMinDuration(minRampTime), and
        % figure out the rest automatically. Reverting full speed in 0.5
        % sec is pretty fast.
        velocities = [chassisCmd.vel; chassisDesired.vel];
        accelerations = [chassisCmd.accel; zeros(1,3) ];
        jerks = [chassisCmd.jerk; zeros(1,3) ];
        
        chassisTraj = trajGen.CHASSIS.newJointMove( velocities, ...
            'Velocities', accelerations, ...
            'Accelerations', jerks, ...
            'Time', time );
        chassisTrajStartTime = timeNow;

        % Spine
        positions = [ spineCmd.pos; spineDesired.pos ];
        velocities = [ spineCmd.vel; spineDesired.vel ];
        accelerations = [ spineCmd.accel; zeros(1,2) ];
        
        spineTraj = trajGen.SPINE.newJointMove( positions, ...
            'Velocities', velocities, ...
            'Accelerations', accelerations, ...
            'Time', time );
        spineTrajStartTime = timeNow;
        
        % Arms
        for i = 1:param.numArms
            
            positions = [ armCmd{i}.pos; armDesired{i}.pos ];
            velocities = [ armCmd{i}.vel; armDesired{i}.vel ];
            accelerations = [ armCmd{i}.accel; zeros(1,4) ];
            
            armTraj{i} = trajGen.ARMS{i}.newJointMove( positions, ...
                'Velocities', velocities, ...
                'Accelerations', accelerations, ...
                'Time', time );
            armTrajStartTime{i} = timeNow;
            
        end
        
        %%%%%%%%%%%%%%%%%
        % APPEND TO LOG %
        %%%%%%%%%%%%%%%%%
        if logging
            timeHist(end+1,1) = timeNow;
            RPYHist(end+1,:) = RPY;
        end
    end
    pause(1.0);
end
    
if logging
    log = struct(robotGroup.stopLogFull());
    timeHist = timeHist - timeHist(1);  
end
