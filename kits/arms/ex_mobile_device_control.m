% 6-dof arm & gripper tele-op Demo
%
% Features:      iOS App input to control motions of arm and gripper
%
% Requirements:  MATLAB 2013b or higher
%
% Author:        Dave Rollinson
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
% Date:          June 2018

% Copyright 2017-2018 HEBI Robotics

%% Setup
clear *;
close all;

% Optional step to limit the lookup to a set of interfaces or modules
% HebiLookup.setLookupAddresses('10.10.10.255');

enableLogging = false;
enableEffortComp = true;


%%
%%%%%%%%%%%%%%%%%%%%%%%
% Mobile Device Setup %
%%%%%%%%%%%%%%%%%%%%%%%
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
%%%%%%%%%%%%%
% Arm Setup %
%%%%%%%%%%%%%

[ robotGroup, armKin, armParams ] = setupArm('6dof_w_gripper');

ikSeedPos = armParams.ikSeedPos;
armEffortOffset = armParams.effortOffset;
gravityVec = [0 0 -1];
numArmDOFs = armKin.getNumDoF();

gripperForceScale = abs(armParams.gripperCloseEffort); 

numModules = robotGroup.getNumModules;

armDOFs = 1:numModules;
gripperDOF = numModules + 1;

% Trajectory
armTrajGen = HebiTrajectoryGenerator(armKin);
armTrajGen.setMinDuration(1.0); % Min move time for 'small' movements
                             % (default is 1.0)
armTrajGen.setSpeedFactor(1.0); % Slow down movements to a safer speed.
                             % (default is 1.0)


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
    xyzTarget_init = [0.3 0.0 0.2];
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
    % CURRENTLY HACKED ON ALTERNATE FEEDBACK CHANNELS UNTIL API UPDATE.
    R_arKit_init = [ fbkPhone.debug1 fbkPhone.debug2 fbkPhone.debug3;
                     fbkPhone.debug4 fbkPhone.debug5 fbkPhone.debug6;
                     fbkPhone.debug7 fbkPhone.debug8 fbkPhone.debug9 ];

    xyz_arKit_init = [ fbkPhone.position ...
                       fbkPhone.velocity ...
                       fbkPhone.effort ];

    endVelocities = zeros(1, numArmDOFs);
    endAccels = zeros(1, numArmDOFs);

    maxDemoTime = inf; % sec
    demoTimer = tic;

    timeLast = t0;
    armTrajStartTime = t0;

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

        % Pose Informatin for arm Control
        % CURRENTLY HACKED ON ALTERNATE FEEDBACK CHANNELS UNTIL API UPDATE.
        xyzPhoneNew = [ fbkPhone.position ...
                        fbkPhone.velocity ...
                        fbkPhone.effort ];
                    
        if phoneControlScale < .1
            xyz_arKit_init = xyzPhoneNew;
        end

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
            t = timeNow - armTrajStartTime;
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
                
        phoneHz = 10;
        phonePeriod = 1 / phoneHz;
        
        if toc(demoTimer) > phonePeriod
            armTrajStartTime = timeNow;
            demoTimer = tic;

            armTraj = armTrajGen.newJointMove( [pos; ikPosition], ...
                        'Velocities', [vel; endVelocities], ...
                        'Accelerations', [accel; endAccels]);  
        end
                    
               
        % Hold down button 8 to put the arm in a compliant grav-comp mode
        if fbkPhoneIO.b8 == 1
            cmd.position(armDOFs) = nan;
            cmd.velocity(armDOFs) = nan;
        end

        %%%%%%%%%%%%%%%%%%%
        % Gripper Control %
        %%%%%%%%%%%%%%%%%%%
        
        % cmd.effort(gripperDOF) = gripperForceScale * fbkPhoneIO.a6;


        %%%%%%%%%%%%%%%%%
        % Send to robot %
        %%%%%%%%%%%%%%%%%
        robotGroup.send(cmd);

    end

end


