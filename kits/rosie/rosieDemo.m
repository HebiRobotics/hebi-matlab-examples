% Mobile Base with 6-DoF arm & gripper.  Tele-op using HEBI Mobile I/O App.
%
% Dave Rollinson
% July 2018

%%
function rosieDemo( mobileBaseType )

    HebiLookup.initialize();

    % Optional step to limit the lookup to a set of interfaces or modules
    % HebiLookup.setLookupAddresses('10.10.10.255');
    enableLogging = false;

    %%
    %%%%%%%%%%%%%%%%%%%%%
    % Setup Mobile Base %
    %%%%%%%%%%%%%%%%%%%%%
    switch lower(mobileBaseType)
        case 'omni'
            [chassisParams, chassisTrajGen] = setupOmniBase();
        case 'diff-drive'
            [chassisParams, chassisTrajGen] = setupDiffDriveBase();
        case 'mecanum'
            disp('Mecanum base not yet implemented');
            return;
        otherwise
            disp('Base type not recognized.'); 
            disp('Please choose: OMNI or DIFF-DRIVE');
            return;
    end

    wheelRadius = chassisParams.wheelRadius; 
    wheelBase = chassisParams.wheelBase;  
    maxLinSpeed = chassisParams.maxLinSpeed; 
    maxRotSpeed = chassisParams.maxRotSpeed; 

    chassisCoM = chassisParams.chassisCoM;  
    chassisMass = chassisParams.chassisMass; 
    chassisInertiaZZ = chassisParams.chassisInertiaZZ; 
    
    chassisMassMatrix = diag( [chassisMass; chassisMass; chassisInertiaZZ] );

    % Maps XYZ chassis velocities to wheel velocities
    chassisToWheelVelocities = chassisParams.wheelVelocityMatrix;
    chassisEffortsToWheelEfforts = chassisParams.wheelEffortMatrix;


    %%
    %%%%%%%%%%%%%
    % Setup Arm %
    %%%%%%%%%%%%%
    [ armParams, armKin, armTrajGen ] = setupArmWithGripper();
    ikSeedPos = armParams.ikSeedPos;
    armEffortOffset = armParams.effortOffset;

    gripperForceScale = abs(armParams.gripperCloseEffort); 

    % Assume that gravity points up
    gravityVec = [0 0 -1];
    numArmDOFs = armKin.getNumDoF();


    %%
    %%%%%%%%%%%%%%%
    % Setup Robot %
    %%%%%%%%%%%%%%%
    robotFamily = 'Rosie';
    robotModuleNames = [ chassisParams.wheelModuleNames, ...
                         armParams.armModuleNames, ...
                         armParams.gripperModuleNames ];
    numModules = length(robotModuleNames);

    wheelDOFs = 1:chassisParams.numWheels;
    armDOFs = wheelDOFs(end) + (1:numArmDOFs);
    gripperDOF = armDOFs(end) + 1;

    robotGroup = HebiLookup.newGroupFromNames( robotFamily, ...
                                               robotModuleNames);
    robotGroup.setFeedbackFrequency(100);
    
    % Set the gains on just the wheels
    wheelGroup = HebiLookup.newGroupFromNames( robotFamily, ...
                                               chassisParams.wheelModuleNames );
    wheelGroup.send( 'gains', chassisParams.wheelGains );
    clear wheelGroup;
    
    % Set the gains on just the arm
    armGroup = HebiLookup.newGroupFromNames( robotFamily, ...
                                             armParams.armModuleNames );
    armGroup.send( 'gains', armParams.armGains );
    clear armGroup;
    
    % Set the gains on just the gripper
    gripperGroup = HebiLookup.newGroupFromNames( robotFamily, ...
                                             armParams.gripperModuleNames );
    gripperGroup.send( 'gains', armParams.gripperGains );
    clear gripperGroup;
    

    %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    % Setup Mobile I/O Group %
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    phoneFamily = 'HEBI';
    phoneName = 'Virtual IO';

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
    
    % Get the initial feedback objects that we'll reuse later
    fbkPhoneIO = phoneGroup.getNextFeedbackIO();
    fbkPhoneMobile = phoneGroup.getNextFeedbackMobile();
    
    %%
    %%%%%%%%%%%%%%%%%%%%%%%
    % Begin the demo loop %
    %%%%%%%%%%%%%%%%%%%%%%%
    
    % This outer loop is what we fall back to anytime we 're-home' the arm
    while true
        fbk = robotGroup.getNextFeedbackFull();

        cmd = CommandStruct();
        cmd.position = nan(size(fbk.position));
        cmd.velocity = nan(size(fbk.position));
        cmd.effort = nan(size(fbk.position));

        xyzScale = [1; 1; 2];

        % Start background logging
        if enableLogging
            robotGroup.startLog();
            phoneGroup.startLog();
        end

        % Move to current coordinates
        xyzTarget_init = [0.3; 0.0; 0.3];
        rotMatTarget_init = R_y(pi);  % Gripper down

        ikPosition = armKin.getIK( 'xyz', xyzTarget_init, ...
                                   'so3', rotMatTarget_init, ...
                                   'initial', ikSeedPos );

        armTrajGen.setSpeedFactor( 0.5 );   

        armTraj = armTrajGen.newJointMove([fbk.position(armDOFs); ikPosition]);

        armTrajGen.setSpeedFactor( armParams.defaultSpeedFactor );   
        armTrajGen.setMinDuration( armParams.minTrajDuration );  

        t0 = fbk.time;
        t = 0;

        while t < armTraj.getDuration()

            fbk = robotGroup.getNextFeedbackFull();
            t = min(fbk.time - t0,armTraj.getDuration);

            [pos,vel,accel] = armTraj.getState(t);
            cmd.position(armDOFs) = pos;
            cmd.velocity(armDOFs) = vel;

            dynamicsComp = armKin.getDynamicCompEfforts( ...
                                    fbk.position(armDOFs), pos, vel, accel );
            gravComp = armKin.getGravCompEfforts( ...
                                    fbk.position(armDOFs), gravityVec );
            cmd.effort(armDOFs) = dynamicsComp + gravComp + armEffortOffset;


            robotGroup.send(cmd);

        end

        % Grab initial pose
        latestPhoneMobile = phoneGroup.getNextFeedback('view','mobile');

        q = [ latestPhoneMobile.arOrientationW ...
              latestPhoneMobile.arOrientationX ...
              latestPhoneMobile.arOrientationY ...
              latestPhoneMobile.arOrientationZ ];     
        R_init = HebiUtils.quat2rotMat( q );

        xyz_init = [ latestPhoneMobile.arPositionX;
                     latestPhoneMobile.arPositionY; 
                     latestPhoneMobile.arPositionZ ];

        xyzPhoneNew = xyz_init;

        endVelocities = zeros(1, numArmDOFs);
        endAccels = zeros(1, numArmDOFs);

        t0 = fbk.time;
        timeLast = t0;
        chassisTrajStartTime = t0;

        wheelCmd.pos = fbk.position(wheelDOFs)';

        % Replan Trajectory for the mobile base
        omniBaseTrajTime = [0 chassisParams.rampTime];

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
            catch
                disp('Could not get robot feedback!');
                break;
            end

            % Get feedback with a timeout of 0, which means that they return
            % instantly, but if there was no new feedback, they return empty.
            % This is because the mobile device is on wireless and might drop
            % out or be really delayed, in which case we would rather keep
            % running with an old data instead of waiting here for new data.
            tempFbk = phoneGroup.getNextFeedback( fbkPhoneIO, fbkPhoneMobile, ...
                                                  'timeout', 0 );
            if ~isempty(tempFbk)
                latestPhoneMobile = fbkPhoneMobile;
                latestPhoneIO = fbkPhoneIO;
            end

            timeNow = fbk.time;
            dt = timeNow - timeLast;
            timeLast = fbk.time;

            % Reset the Command Struct
            cmd.effort = nan(1,numModules);
            cmd.position = nan(1,numModules);
            cmd.velocity = nan(1,numModules);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Read/Map Joystick Inputs %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Check for restart command
            if latestPhoneIO.b1
                break;
            end

            % Parameter to limit XYZ Translation of the arm if a slider is
            % pulled down.
            phoneControlScale = (latestPhoneIO.a3 + 1) / 2 ;
            if phoneControlScale < .1
                xyz_init = xyzPhoneNew;
            end

            % Joystick Input for Omnibase Control        
            xVel = maxLinSpeed * latestPhoneIO.a8; % Right Pad Up/Down
            yVel = -maxLinSpeed * latestPhoneIO.a7; % Right Pad Left/Right 
            rotVel = maxRotSpeed * latestPhoneIO.a1; % Left Pad Left/Right 

            % Pose Information for Arm Control
            xyzPhoneNew = [ latestPhoneMobile.arPositionX; ...
                            latestPhoneMobile.arPositionY; ...
                            latestPhoneMobile.arPositionZ ];

            xyzTarget = xyzTarget_init + phoneControlScale * xyzScale .* ...
                            (R_init' * (xyzPhoneNew - xyz_init));

            q = [ latestPhoneMobile.arOrientationW ...
                  latestPhoneMobile.arOrientationX ...
                  latestPhoneMobile.arOrientationY ...
                  latestPhoneMobile.arOrientationZ ];     
            rotMatTarget = R_init' * HebiUtils.quat2rotMat( q ) * rotMatTarget_init;

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

            dynamicsComp = armKin.getDynamicCompEfforts( ...
                                fbk.position(armDOFs), pos, vel, accel );
            gravComp = armKin.getGravCompEfforts( ...
                                fbk.position(armDOFs), gravityVec );
            cmd.effort(armDOFs) = dynamicsComp + gravComp + armEffortOffset;

            % Force elbow up config
            seedPosIK = pos;
            seedPosIK(3) = abs(seedPosIK(3));

            % Find target using inverse kinematics
            ikPosition = armKin.getIK( 'xyz', xyzTarget, ...
                                       'SO3', rotMatTarget, ...
                                       'initial', seedPosIK, ...
                                       'MaxIter', 50 ); 

            % Start new trajectory at the current state
            armTraj = armTrajGen.newJointMove( [pos; ikPosition], ...
                            'Velocities', [vel; endVelocities], ...
                            'Accelerations', [accel; endAccels]);  

            %%%%%%%%%%%%%%%%%%%
            % Gripper Control %
            %%%%%%%%%%%%%%%%%%%
            cmd.effort(gripperDOF) = gripperForceScale * latestPhoneIO.a6;


            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Evaluate Trajectory State %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Chassis (linear velocity)
            t = min(timeNow - chassisTrajStartTime, omniBaseTraj.getDuration);
            [chassisCmd.vel, chassisCmd.accel, chassisCmd.jerk] = ...
                omniBaseTraj.getState( t );

            % Chassis (convert linear to joint velocities)
            wheelCmd.vel = chassisToWheelVelocities * chassisCmd.vel';
            wheelCmd.pos = wheelCmd.pos + wheelCmd.vel * dt;
            wheelCmd.effort = chassisEffortsToWheelEfforts * ...
                            (chassisMassMatrix * chassisCmd.accel');

            cmd.position(wheelDOFs) = wheelCmd.pos;
            cmd.velocity(wheelDOFs) = wheelCmd.vel;
            cmd.effort(wheelDOFs) = wheelCmd.effort;   

            % Hold down button 8 to put the arm in a compliant grav-comp mode
            if latestPhoneIO.b8 == 1
                cmd.position(armDOFs) = nan;
                cmd.velocity(armDOFs) = nan;
            end

            % Send to robot
            robotGroup.send(cmd);

            % Chassis (linear velocity)
            chassisDesired.vel = [xVel yVel rotVel];

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

