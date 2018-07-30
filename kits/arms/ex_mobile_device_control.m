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


%%
%%%%%%%%%%%%%
% Arm Setup %
%%%%%%%%%%%%%

[ robotGroup, armKin, armParams ] = setupArm('6dof');

ikSeedPos = armParams.ikSeedPos;
armEffortOffset = armParams.effortOffset;
gravityVec = [0 0 -1];
numArmDOFs = armKin.getNumDoF();

numModules = robotGroup.getNumModules;

armDOFs = 1:numModules;

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

    maxDemoTime = inf; % sec
    phoneFbkTimer = tic;

    timeLast = t0;
    armTrajStartTime = t0;

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
        if fbkPhoneIO.b1
            break;
        end
        
        % Parameter to limit XYZ Translation of the arm if a slider is
        % pulled down.
        phoneControlScale = (fbkPhoneIO.a3 + 1) / 2 ;
        if phoneControlScale < .1
            xyz_init = xyzPhoneNew;
        end

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
        
        if toc(phoneFbkTimer) > phonePeriod
            armTrajStartTime = timeNow;
            phoneFbkTimer = tic;

            armTraj = armTrajGen.newJointMove( [pos; ikPosition], ...
                        'Velocities', [vel; endVelocities], ...
                        'Accelerations', [accel; endAccels]);  
        end

        %%%%%%%%%%%%%%%%%
        % Send to robot %
        %%%%%%%%%%%%%%%%%
        robotGroup.send(cmd);

    end

end


