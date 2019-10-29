% 6-DoF Arm (w/ Optional Gripper) Tele-Op Demo 
%
% Features:      Mobile App input to control motions of arm and gripper
%
% Requirements:  MATLAB 2013b or higher
%                Android or iOS device running HEBI Mobile I/O that
%                supports ARCore / ARKit.
%
% Author:        Dave Rollinson
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
% Date:          Oct 2018

% Copyright 2017-2018 HEBI Robotics

%% Setup
clear *;
close all;

HebiLookup.initialize();

enableLogging = true;
enableEffortComp = true;


%%
%%%%%%%%%%%%%%%%%%%%%%%
% Mobile Device Setup %
%%%%%%%%%%%%%%%%%%%%%%%
phoneFamily = 'HEBI';
phoneName = 'Mobile IO';

resetPoseButton = 'b1';
quitDemoButton = 'b8';
translationScaleSlider = 'a3';
gripForceSlider = 'a6';

abortFlag = false;

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

armName = '5-DoF';
armFamily = 'Arm';
actuatorSeries = 'R-Series';
hasGasSpring = true;  % If you attach a gas spring to the shoulder for
                       % extra payload, set this to TRUE.

[ armGroup, armKin, armParams ] = setupArm( armName, armFamily, ...
                                            actuatorSeries, hasGasSpring );

ikSeedPos = armParams.ikSeedPos;
armEffortOffset = armParams.effortOffset;
gravityVec = armParams.gravityVec;
localDir = armParams.localDir;

if armParams.hasGripper
    gripperGroup = HebiLookup.newGroupFromNames( armFamily, 'Spool' );
    gripperGroup.send( 'gains', armParams.gripperGains );
    gripForceScale = 0.5 * (armParams.gripperOpenEffort - ...
                            armParams.gripperCloseEffort); 
    gripForceShift = mean( [ armParams.gripperOpenEffort, ...
                             armParams.gripperCloseEffort ] ); 
    gripperCmd = CommandStruct();
end

numArmDOFs = armKin.getNumDoF();

numModules = armGroup.getNumModules;

armDOFs = 1:numModules;

% Trajectory
armTrajGen = HebiTrajectoryGenerator(armKin);
armTrajGen.setMinDuration(1.0); % Min move time for 'small' movements
                             % (default is 1.0)
armTrajGen.setSpeedFactor(1.0); % Slow down movements to a safer speed.
                             % (default is 1.0)
                             
disp('  ');
disp('Arm end-effector is now following the mobile device pose.');
disp('The control interface has the following commands:');
disp('  B1 - Reset/re-align poses.');
disp('       This takes the arm to home and aligns with mobile device.');
disp('  A3 - Scale down the translation commands to the arm.');
disp('       Sliding all the way down means the end-effector only rotates.');
disp('  A6 - Control the gripper (if the arm has gripper).');
disp('       Sliding down closes the gripper, sliding up opens.');
disp('  B8 - Quits the demo.');


%% Startup
while ~abortFlag
    fbk = armGroup.getNextFeedbackFull();
    fbkPhoneMobile = phoneGroup.getNextFeedback('view','mobile');
    fbkPhoneIO = phoneGroup.getNextFeedback('view','io');

    cmd = CommandStruct();
    cmd.position = nan(size(fbk.position));
    cmd.velocity = nan(size(fbk.position));
    cmd.effort = nan(size(fbk.position));

    xyzScale = [1 1 2]';

    % Start background logging
    if enableLogging
        armGroup.startLog('dir',[localDir '/logs']);
        phoneGroup.startLog('dir',[localDir '/logs']);
    end

    % Move to current coordinates
    xyzTarget_init = [0.5 0.0 0.1]';
    rotMatTarget_init = R_y(pi);

    ikPosition = armKin.getIK( 'xyz', xyzTarget_init, ...
                               'so3', rotMatTarget_init, ...
                               'initial', ikSeedPos );

    % Slow trajectories down for the initial move to home position                       
    armTrajGen.setSpeedFactor( 0.5 );   
    
    armTraj = armTrajGen.newJointMove([fbk.position(armDOFs); ikPosition]);
    
    % Set trajectories to normal speed for following mobile input
    armTrajGen.setSpeedFactor( 1.0 );   
    armTrajGen.setMinDuration( 0.33 );  

    t0 = fbk.time;
    t = 0;

    while t < armTraj.getDuration()

        fbk = armGroup.getNextFeedbackFull();
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

        armGroup.send(cmd);

    end
    
    % Grab initial pose
    fbkPhoneMobile = phoneGroup.getNextFeedback('view','mobile');
    fbkPhoneIO = phoneGroup.getNextFeedback('view','io');
    latestPhoneMobile = fbkPhoneMobile;
    latestPhoneIO = fbkPhoneIO;

    q = [ fbkPhoneMobile.arOrientationW ...
          fbkPhoneMobile.arOrientationX ...
          fbkPhoneMobile.arOrientationY ...
          fbkPhoneMobile.arOrientationZ ];     
    R_init = HebiUtils.quat2rotMat( q );

    xyz_init = [ fbkPhoneMobile.arPositionX;
                 fbkPhoneMobile.arPositionY; 
                 fbkPhoneMobile.arPositionZ ];

    xyzPhoneNew = xyz_init;

    endVelocities = zeros(1, numArmDOFs);
    endAccels = zeros(1, numArmDOFs);

    maxDemoTime = inf; % sec
    phoneFbkTimer = tic;

    timeLast = t0;
    armTrajStartTime = t0;

    firstRun = true;

    while ~abortFlag

        %%%%%%%%%%%%%%%%%%%
        % Gather Feedback %
        %%%%%%%%%%%%%%%%%%%
        try
            fbk = armGroup.getNextFeedback( 'view', 'full' );
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
        if fbkPhoneIO.(resetPoseButton)
            break;
        end
        
        % Check for quit command
        if fbkPhoneIO.(quitDemoButton)
            abortFlag = true;
            break;
        end
        
        if armParams.hasGripper
            gripperCmd.effort = gripForceScale * ...
                                    latestPhoneIO.(gripForceSlider) + ...
                                    gripForceShift;
            gripperGroup.send(gripperCmd);
        end
        
        % Parameter to limit XYZ Translation of the arm if a slider is
        % pulled down.  Pulling all the way down resets translation.
        phoneControlScale = fbkPhoneIO.(translationScaleSlider);
        if phoneControlScale < 0
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
        armGroup.send(cmd);

    end

end

%%
if enableLogging
    
   hebilog = armGroup.stopLogFull();
   
   % Plot tracking / error from the joints in the arm.  Note that there
   % will not by any 'error' in tracking for position and velocity, since
   % this example only commands effort.
   HebiUtils.plotLogs(hebilog, 'position');
   HebiUtils.plotLogs(hebilog, 'velocity');
   HebiUtils.plotLogs(hebilog, 'effort');
   
   % Plot the end-effectory trajectory and error
   kinematics_analysis( hebilog, armKin );
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   % Feel free to put more plotting code here %
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

