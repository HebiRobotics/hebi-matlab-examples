% Mobile Base with 6-dof arm & gripper tele-op Demo
%
% Features:      iOS App input to control motions of omni-drive mobile base
%                as well as arm and gripper
%
% Requirements:  MATLAB 2013b or higher
%
% Author:        Dave Rollinson
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
% Date:          May 2018

% Copyright 2017-2018 HEBI Robotics

function rosieOmniDriveDemo()
%% Setup

% Optional step to limit the lookup to a set of interfaces or modules
% HebiLookup.setLookupAddresses('10.10.10.255');

enableLogging = false;

enableEffortComp = true;

% Setup Group for the entire Robot
robotFamily = 'Rosie';
robotModuleNames = {
    '_Wheel1', ...   % right front omni wheel
    '_Wheel2', ...   % left front omni wheel
    '_Wheel3', ...   % back middle omni wheel
    'Base', 'Shoulder', 'Elbow', 'Wrist1', 'Wrist2', 'Wrist3', ... % arm modules
    'Spool' };

wheelDOFs = 1:3;
armDOFs = 4:9;
gripperDOF = 10;
numModules = length(robotModuleNames);

robotGroup = HebiLookup.newGroupFromNames( robotFamily, robotModuleNames);
robotGroup.setFeedbackFrequency(100);

% Setup Group for phone controller
phoneFamily = 'HEBI';
phoneName = 'Mobile IO - Matt iPad';

while true        
    try
        fprintf('Searching for phone Controller...\n');
        phoneGroup = HebiLookup.newGroupFromNames( ...
                        phoneFamily, phoneName );        
        disp('Phone Found.  Starting up');
        break;
    catch
        pause(1.0);
    end

end


%%  
%%%%%%%%%%%%%%%%%%%%%%%
% Omni Base Kinematics %
%%%%%%%%%%%%%%%%%%%%%%%%

[chassisParams, chassisTrajGen] = setupOmniBase();

wheelRadius = chassisParams.wheelRadius;  % m
wheelBase = chassisParams.wheelBase;   % m (center of omni to origin of base)
maxLinSpeed = chassisParams.maxLinSpeed; % m/s
maxRotSpeed = chassisParams.maxRotSpeed; % rad/s

chassisCoM = chassisParams.chassisCoM;  % m
chassisMass = chassisParams.chassisMass;  % kg

chassisMassMatrix = diag( [chassisMass; chassisMass; 0] );

% Maps XYZ chassis velocities to wheel velocities
chassisToWheelVelocities = chassisParams.wheelVelocityMatrix;
chassisEffortsToWheelEfforts = chassisParams.wheelEffortMatrix;

%%
%%%%%%%%%%%%%%%%%%
% Arm Kinematics %
%%%%%%%%%%%%%%%%%%

[ armParams, armKin, armTrajGen ] = setupArmWithGripper();
ikSeedPos = armParams.ikSeedPos;
armEffortOffset = armParams.effortOffset;

gravityVec = [0 0 -1];
numArmDOFs = armKin.getNumDoF();

%% Startup
while true
    fbk = robotGroup.getNextFeedbackFull();
    fbkPhone = phoneGroup.getNextFeedback('view','debug');
    fbkPhoneIO = phoneGroup.getNextFeedback('view','io');

    cmd = CommandStruct();
    cmd.position = nan(size(fbk.position));
    cmd.velocity = nan(size(fbk.position));
    cmd.effort = nan(size(fbk.position));

    xyzScale = [1 1 2];

    % Start background logging
    if enableLogging
        robotGroup.startLog();
        phoneGroup.startLog();
    end

    % Move to current coordinates
    xyzTarget_init = [0.3 0.0 0.3];
    rotMatTarget_init = R_y(pi);

    q = [ fbkPhone.orientationW ...
          fbkPhone.orientationX ...
          fbkPhone.orientationY ...
          fbkPhone.orientationZ ];     
    R_coreMotion_init = HebiUtils.quat2rotMat( q );

    q = [ fbk.orientationW( armDOFs(1) ) ...
          fbk.orientationX( armDOFs(1) ) ...
          fbk.orientationY( armDOFs(1) ) ...
          fbk.orientationZ( armDOFs(1) ) ];     
    R_base_init = HebiUtils.quat2rotMat( q );


    ikPosition = armKin.getIK( 'xyz', xyzTarget_init, ...
                               'so3', rotMatTarget_init, ...
                               'initial', ikSeedPos );

    armTrajGen.setSpeedFactor( 0.5 );   
    %armTrajGen.setMinDuration( 1.0 );  
    
    armTraj = armTrajGen.newJointMove([fbk.position(armDOFs); ikPosition]);
    
    armTrajGen.setSpeedFactor( 0.9 );   
    armTrajGen.setMinDuration( 0.33 );  

    t0 = fbk.time;
    t = 0;

    while t < armTraj.getDuration()

        fbk = robotGroup.getNextFeedbackFull();
        t = min(fbk.time - t0,armTraj.getDuration);

        [pos,vel,accel] = armTraj.getState(t);
        cmd.position(armDOFs) = pos;
        cmd.velocity(armDOFs) = vel;

        if enableEffortComp
            dynamicsComp = armKin.getDynamicCompEfforts( ...
                                    fbk.position(armDOFs), pos, vel, accel );
            gravComp = armKin.getGravCompEfforts( ...
                                    fbk.position(armDOFs), gravityVec );
            cmd.effort(armDOFs) = dynamicsComp + gravComp + armEffortOffset;
        end

        robotGroup.send(cmd);

    end
    
    fbkPhone = phoneGroup.getNextFeedback('view','debug');
    
    % Grab initial pose
    R_arKit_init = [ fbkPhone.debug1 fbkPhone.debug2 fbkPhone.debug3;
                     fbkPhone.debug4 fbkPhone.debug5 fbkPhone.debug6;
                     fbkPhone.debug7 fbkPhone.debug8 fbkPhone.debug9 ];

    xyz_arKit_init = [ fbkPhone.position ...
                       fbkPhone.velocity ...
                       fbkPhone.effort ];
    xyzPhoneNew = xyz_arKit_init;

    endVelocities = zeros(1, numArmDOFs);
    endAccels = zeros(1, numArmDOFs);

    t0 = fbk.time;

    maxDemoTime = inf; % sec
    demoTimer = tic;

    timeLast = t0;
    chassisTrajStartTime = t0;

    wheelCmd.pos = fbk.position(wheelDOFs)';

    % Replan Trajectory for the mobile base
    rampTime = 0.33;
    omniBaseTrajTime = [0 rampTime];

    % Initialize trajectory for Omnibase
    velocities = zeros(2,3);
    accelerations = zeros(2,3);
    jerks = zeros(2,3);

    omniBaseTraj = chassisTrajGen.newJointMove( velocities, ...
        'Velocities', accelerations, ...
        'Accelerations', jerks, ...
        'Time', omniBaseTrajTime );

    firstRun = true;

    while true

        %%%%%%%%%%%%%%%%%%%
        % Gather Feedback %
        %%%%%%%%%%%%%%%%%%%
        try
            fbk = robotGroup.getNextFeedback( 'view', 'full' );
            %fbkPhone = phoneGroup.getNextFeedback( 'view', 'debug' );
            %fbkPhoneIO = phoneGroup.getNextFeedback( 'view', 'io' );
        catch
            disp('Could not get robot feedback!');
            break;
        end

        try
            fbkPhone = phoneGroup.get( 'feedback', 'view', 'debug' );
            fbkPhoneIO = phoneGroup.get( 'feedback', 'view', 'io' );
        catch
            disp('Could not get phone feedback!');
            break;
        end

        timeNow = fbk.time;
        dt = timeNow - timeLast;
        timeLast = fbk.time;

        % Reset the Command Struct
        cmd.effort = nan(1,numModules);
        cmd.position = nan(1,numModules);
        cmd.velocity = nan(1,numModules);

        % Update base orientation
        q = [ fbk.orientationW(1) ...
              fbk.orientationX(1) ...
              fbk.orientationY(1) ...
              fbk.orientationZ(1) ];
        R_base = R_base_init' * HebiUtils.quat2rotMat( q );


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Read/Map Joystick Inputs %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % Check for restart command
        if fbkPhoneIO.b1
            break;
        end
        
        % Parameter to limit XYZ Translation of the arm if a slider is
        % pulled down.
        phoneControlScale = (fbkPhoneIO.a3 + 1) / 2 ;
        if phoneControlScale < .1
            xyz_arKit_init = xyzPhoneNew;
        end
            
%         % Joystick Input for Omnibase Control
%         % I THINK THERE'S A BUG IN CONVENTION FOR X-Y, NEED TO CHECK.
%         xVel = maxLinSpeed * fbkPhoneIO.a7;
%         yVel = maxLinSpeed * fbkPhoneIO.a8;
%         rotVel = maxRotSpeed * fbkPhoneIO.a1;
        
        xVel = maxLinSpeed * fbkPhoneIO.a8; % Right Pad Up/Down
        yVel = -maxLinSpeed * fbkPhoneIO.a7; % Right Pad Left/Right 
        rotVel = maxRotSpeed * fbkPhoneIO.a1; % Left Pad Up/Down

        % Pose Information for arm Control
        % CURRENTLY HACKED ON POS/VEL/EFFORT FEEDBACK UNTIL API UPDATE.
        xyzPhoneNew = [ fbkPhone.position ...
                        fbkPhone.velocity ...
                        fbkPhone.effort ];

        xyzTarget = xyzTarget_init + phoneControlScale * xyzScale .* ...
                        (R_z(-pi/2) * R_arKit_init' * (xyzPhoneNew' - xyz_arKit_init'))';

        q = [ fbkPhone.orientationW ...
              fbkPhone.orientationX ...
              fbkPhone.orientationY ...
              fbkPhone.orientationZ ];     
        rotMatTarget = R_coreMotion_init' * HebiUtils.quat2rotMat( q ) * rotMatTarget_init;

        %%%%%%%%%%%%%%%
        % Arm Control %
        %%%%%%%%%%%%%%%
        % Get state of current trajectory
        if firstRun
            pos = fbk.positionCmd(armDOFs);
            vel = endVelocities;
            accel = endAccels;
            firstRun = false;
        else
            [pos,vel,accel] = armTraj.getState(t);
        end

        cmd.position(armDOFs) = pos;
        cmd.velocity(armDOFs) = vel;

        if enableEffortComp
            dynamicsComp = armKin.getDynamicCompEfforts( ...
                                fbk.position(armDOFs), pos, vel, accel );
            gravComp = armKin.getGravCompEfforts( ...
                                fbk.position(armDOFs), gravityVec );
            cmd.effort(armDOFs) = dynamicsComp + gravComp + armEffortOffset;
        end 

        % Force elbow up config
        seedPosIK = pos;
        seedPosIK(3) = abs(seedPosIK(3));

        % Find target using inverse kinematics
        ikPosition = armKin.getIK( 'xyz', xyzTarget, ...
                                   'SO3', rotMatTarget, ...
                                   'initial', seedPosIK, ...
                                   'MaxIter', 50 ); 

        % Start new trajectory at the current state
        t0 = fbk.time;
        armTraj = armTrajGen.newJointMove( [pos; ikPosition], ...
                        'Velocities', [vel; endVelocities], ...
                        'Accelerations', [accel; endAccels]);  

        %%%%%%%%%%%%%%%%%%%
        % Gripper Control %
        %%%%%%%%%%%%%%%%%%%
        cmd.effort(gripperDOF) = 5 * fbkPhoneIO.a6;

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Evaluate Trajectory State %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Chassis (linear velocity)
        t = min(timeNow - chassisTrajStartTime, omniBaseTraj.getDuration);
        [chassisCmd.vel, chassisCmd.accel, chassisCmd.jerk] = ...
            omniBaseTraj.getState( t );
      
        % Chassis (convert linear to joint velocities)
        % Note that the maximum desired linear velocity is already limited, 
        % so we don't need to separately check whether the joint velocities 
        % along the trajectory are within achievable limits.
        wheelCmd.vel = chassisToWheelVelocities * chassisCmd.vel';
        wheelCmd.pos = wheelCmd.pos + wheelCmd.vel * dt;
        
        wheelCmd.effort = chassisEffortsToWheelEfforts * ...
                        (chassisMassMatrix * chassisCmd.accel');

        cmd.position(wheelDOFs) = wheelCmd.pos;
        cmd.velocity(wheelDOFs) = wheelCmd.vel;
        cmd.effort(wheelDOFs) = wheelCmd.effort;   
        
        % Hold down button 8 to put the arm in a compliant grav-comp mode
        if fbkPhoneIO.b8 == 1
            cmd.position(armDOFs) = nan;
            cmd.velocity(armDOFs) = nan;
        end

        % Send to robot
        robotGroup.send(cmd);

        % Chassis (linear velocity)
        chassisDesired.vel = [xVel; yVel; rotVel]';

        velocities = [chassisCmd.vel; chassisDesired.vel];
        accelerations = [chassisCmd.accel; zeros(1,3) ];
        jerks = [chassisCmd.jerk; zeros(1,3) ];

        omniBaseTraj = chassisTrajGen.newJointMove( velocities, ...
            'Velocities', accelerations, ...
            'Accelerations', jerks, ...
            'Time', omniBaseTrajTime );
        chassisTrajStartTime = timeNow;

    end

end
end

