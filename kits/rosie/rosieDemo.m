% Mobile Base with 6-DoF arm & gripper.  Tele-op using HEBI Mobile I/O App.
%
% Dave Rollinson
% July 2018

%%
function rosieDemo( mobileBaseType )

    HebiLookup.initialize();

    % Optional step to limit the lookup to a set of interfaces or modules
    % HebiLookup.setLookupAddresses('10.10.10.255');
    enableLogging = true;

    %%
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Setup Arm and Gripper %
    %%%%%%%%%%%%%%%%%%%%%%%%%
    robotFamily = 'Rosie';
    [ arm, armParams, gripper ] = setupArmWithGripper(robotFamily);
    
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
            [chassisParams, chassisTrajGen] = setupMecanumBase();
        otherwise
            disp('Base type not recognized.'); 
            disp('Please choose: OMNI, DIFF-DRIVE, or MECANUM');
            return;
    end

    % Not used anymore? TODO: remove?
% %     wheelRadius = chassisParams.wheelRadius; 
% %     wheelBase = chassisParams.wheelBase;  
% %     chassisCoM = chassisParams.chassisCoM;  
   
    % Max speed
    maxLinSpeed = chassisParams.maxLinSpeed; 
    maxRotSpeed = chassisParams.maxRotSpeed; 
    
    % Maps linear (XYZ) chassis velocities to wheel velocities
    chassisToWheelVelocities = chassisParams.wheelVelocityMatrix;
    chassisEffortsToWheelEfforts = chassisParams.wheelEffortMatrix;
    chassisMass = chassisParams.chassisMass; 
    chassisInertiaZZ = chassisParams.chassisInertiaZZ; 
    chassisMassMatrix = diag( [chassisMass; chassisMass; chassisInertiaZZ] );

    % Create Group
    wheelGroup = HebiLookup.newGroupFromNames(robotFamily, chassisParams.wheelModuleNames);
    HebiUtils.sendWithRetry(wheelGroup, 'gains', chassisParams.wheelGains);
    wheelCmd = CommandStruct();

    %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Setup Mobile Phone Input %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    phoneFamily = 'Rosie';
    phoneName = 'mobileIO';
    
    resetPoseButton = 'b1';
    compliantModeButton = 'b2';
    quitDemoButton = 'b8';
    translationScaleSlider = 'a3';
    gripForceSlider = 'a6';
    
    xVelAxis = 'a8'; % Right Pad Up/Down
    yVelAxis = 'a7'; % Right Pad Left/Right
    rotVelAxis = 'a1'; % Left Pad Left/Right
    
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
    % Begin the demo loop %
    %%%%%%%%%%%%%%%%%%%%%%%
    
    % Start background logging
    if enableLogging
        arm.group.startLog('dir','logs');
        wheelGroup.startLog('dir','logs');
    end
    
    % This outer loop is what we fall back to anytime we 're-home' the arm
    abortFlag = false;
    while ~abortFlag

        % Exaggerate Z-Axis by 2x, X-Y are 1-to-1. 
        xyzScale = [1 1 2]';
        
        % Move to current coordinates
        xyzTarget_init = [0.3; 0.0; 0.3];
        rotMatTarget_init = R_y(-pi);  % Gripper down
        
        ikPosition = arm.kin.getIK(...
            'xyz', xyzTarget_init, ...
            'so3', rotMatTarget_init, ...
            'initial', armParams.ikSeedPos );
        
        % Slow trajectory timing for the initial move to home position
        arm.trajGen.setSpeedFactor( 0.5 );
        arm.trajGen.setMinDuration( 1.0 );
        
        % Move to initial position
        arm.update();
        arm.clearGoal(); % in case we run only this section
        arm.setGoal(ikPosition);
        while ~arm.isAtGoal
            arm.update();
            arm.send();
        end
        
        % Reset behavior to normal speed
        arm.trajGen.setSpeedFactor(armParams.defaultSpeedFactor);
        arm.trajGen.setMinDuration(armParams.minTrajDuration);
       
        % Grab initial pose from phone
        fbkPhoneMobile = phoneGroup.getNextFeedbackMobile();
        fbkPhoneIO = phoneGroup.getNextFeedbackIO();
        
        q = [ fbkPhoneMobile.arOrientationW ...
              fbkPhoneMobile.arOrientationX ...
              fbkPhoneMobile.arOrientationY ...
              fbkPhoneMobile.arOrientationZ ];     
        R_init = HebiUtils.quat2rotMat( q );

        xyz_init = [ fbkPhoneMobile.arPositionX;
                     fbkPhoneMobile.arPositionY; 
                     fbkPhoneMobile.arPositionZ ];
        xyzPhoneNew = xyz_init;

        
        % Initialize a trajectory for the base. (By passing velocities 
        % as positions we can compute a velocity trajectory)
        timeNow = arm.state.time;
        chassisTrajStartTime = timeNow;
        
        velocities = zeros(2, 3);
        chassisTraj = chassisTrajGen.newJointMove( velocities, ...
            'Time', [0 chassisParams.rampTime]);
        
        % Initialize wheel position to current feedback and then
        % integrate from there.
        wheelFbk = wheelGroup.getNextFeedback();
        wheelCmd.position = wheelFbk.position;

        while ~abortFlag

            %%%%%%%%%%%%%%%%%%%
            % Gather Feedback %
            %%%%%%%%%%%%%%%%%%%
            try
                arm.update();
                arm.send();
                if ~isempty(gripper)
                    gripper.send();
                end
                
                % Wheel feedback. Uncomment if used
                % wheelFbk = wheelGroup.getNextFeedback();
            catch
                disp('Could not get robot feedback!');
                break;
            end 

            % We get feedback from the phone into the existing structs. The
            % timeout of 0 means that the method returns immediately and won't
            % wait for feedback. If there was no feedback, the method returns
            % empty ([]), and the data in the passed in structs does not get
            % overwritten.
            % We do this because the mobile device is typically on wireless and
            % might drop out or be really delayed, in which case we would
            % rather keep running with an old data instead of waiting here for
            % new data.
            hasNewPhoneFbk = ~isempty(phoneGroup.getNextFeedback( ...
                fbkPhoneIO, fbkPhoneMobile, ... % overwrite existing structs
                'timeout', 0 )); % prevent blocking due to bad comms
            

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Read/Map Joystick Inputs %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Check for restart command
            if fbkPhoneIO.(resetPoseButton)
                break;
            end
            
            % Check for quit command
            if fbkPhoneIO.(quitDemoButton)
                abortFlag = true;
                break;
            end
            
            % Parameter to limit XYZ Translation of the arm if a slider is
            % pulled down.
            phoneControlScale = (fbkPhoneIO.(translationScaleSlider) + 1) / 2 ;
            if phoneControlScale < .1
                xyz_init = xyzPhoneNew;
            end
            
            
            %%%%%%%%%%%%%%%%%%%
            % Gripper Control %
            %%%%%%%%%%%%%%%%%%%
            % Map [-1,+1] slider to [0,1] range such that down is close
            if ~isempty(gripper)
                gripper.setState((fbkPhoneIO.(gripForceSlider) - 1) / -2);
            end

            
            %%%%%%%%%%%%%%%
            % Arm Control %
            %%%%%%%%%%%%%%%
            % Convert AR transform to target
            xyzPhoneNew = [ fbkPhoneMobile.arPositionX; ...
                            fbkPhoneMobile.arPositionY; ...
                            fbkPhoneMobile.arPositionZ ];
            xyzTarget = xyzTarget_init + phoneControlScale * xyzScale .* ...
                            (R_init' * (xyzPhoneNew - xyz_init));
                        
            q = [ fbkPhoneMobile.arOrientationW ...
                  fbkPhoneMobile.arOrientationX ...
                  fbkPhoneMobile.arOrientationY ...
                  fbkPhoneMobile.arOrientationZ ];     
            rotMatTarget = R_init' * HebiUtils.quat2rotMat( q ) * rotMatTarget_init;
            
             if fbkPhoneIO.(compliantModeButton) == 1
                 % Enable mode that keeps arm in a compliant grav-comp
                 arm.clearGoal();
                 seedPosIK = armParams.ikSeedPos;
             else
                 
                 % Force elbow up config
                 if ~isempty(arm.state.cmdPos)
                     seedPosIK = arm.state.cmdPos;
                     seedPosIK(3) = abs(seedPosIK(3));
                 end
                 
                 % Find target using inverse kinematics
                 ikPosition = arm.kin.getIK('xyz', xyzTarget, ...
                     'SO3', rotMatTarget, ...
                     'initial', seedPosIK, ...
                     'MaxIter', 50 );
             
                 % Plan a trajectrory towards the goal
                 arm.setGoal(ikPosition);
             end
             
             
            %%%%%%%%%%%%%%%%%%%%%%%
            % Mobile Base Control %
            %%%%%%%%%%%%%%%%%%%%%%%
            % Map joystick input to linear speeds       
            xVel = maxLinSpeed * fbkPhoneIO.(xVelAxis);
            yVel = -maxLinSpeed * fbkPhoneIO.(yVelAxis);
            rotVel = maxRotSpeed * fbkPhoneIO.(rotVelAxis); 
            desiredChassisVel = [xVel yVel rotVel];
            
            % Find the current point in chassis trajectory
            % (linear velocity) we plan a smooth transition
            t = min(timeNow - chassisTrajStartTime, chassisTraj.getDuration);
            [chassisVel, chassisAccel, chassisJerk] = chassisTraj.getState(t); 
            
            % Compute a trajectory that smoothly transitions to 
            % the new desired velocities
            velocities = [chassisVel; desiredChassisVel];
            accelerations = [chassisAccel; zeros(1,3) ];
            jerks = [chassisJerk; zeros(1,3) ];
            chassisTraj = chassisTrajGen.newJointMove( velocities, ...
                'Velocities', accelerations, ...
                'Accelerations', jerks, ...
                'Time', [0 chassisParams.rampTime] );
            chassisTrajStartTime = timeNow;
            
            % Compute dt since the last update for integrating position
            timeLast = timeNow;
            timeNow = arm.state.time;
            dt = timeNow - timeLast;
            
            % Convert linear velocities into wheel/joint velocities
            wheelCmd.velocity = (chassisToWheelVelocities * chassisVel')';
            wheelCmd.position = wheelCmd.position + wheelCmd.velocity * dt;
            wheelCmd.effort = (chassisEffortsToWheelEfforts * ...
                            (chassisMassMatrix * chassisAccel'))';
            wheelGroup.send(wheelCmd);
            
        end
    end
end

