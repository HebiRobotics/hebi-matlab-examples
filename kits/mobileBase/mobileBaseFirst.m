% Mobile Base with 6-DoF arm & gripper.  Tele-op using HEBI Mobile I/O App.
%
% Dave Rollinson
% July 2018

%%
function mobileBaseFirst( mobileBaseType )

HebiLookup.initialize();

% Optional step to limit the lookup to a set of interfaces or modules
% HebiLookup.setLookupAddresses('10.10.10.255');
enableLogging = true;
robotFamily = 'Chevron';

phoneCmd = IoCommandStruct();

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
    case 'pneum'
        [chassisParams, chassisTrajGen] = setupPneumDriveBase();
    otherwise
        disp('Base type not recognized.');
        disp('Please choose: OMNI, DIFF-DRIVE, or MECANUM');
        return;
end

% Max speed
maxLinSpeed = chassisParams.maxLinSpeed;
maxRotSpeed = chassisParams.maxRotSpeed;

% Maps linear (XYZ) chassis velocities to wheel velocities
chassisToWheelVelocities = chassisParams.wheelVelocityMatrix;
chassisEffortsToWheelEfforts = chassisParams.wheelEffortMatrix;
chassisMass = chassisParams.chassisMass;
chassisInertiaZZ = chassisParams.chassisInertiaZZ;
chassisMassMatrix = diag( [chassisMass; chassisInertiaZZ] );

%Create Drive Group
driveNames =  chassisParams.driveModuleNames;

driveGroup = HebiLookup.newGroupFromNames(robotFamily, driveNames);
HebiUtils.sendWithRetry(driveGroup, 'gains', chassisParams.wheelGains);
driveCmd = CommandStruct();


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % Setup Camera%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
camNames = {'Widey'};
camGroup = HebiLookup.newGroupFromNames( robotFamily, camNames);
camKin = HebiKinematics('hrdf/Chevron-CamTail');
camCmdIO = IoCommandStruct();
floodLED = 'f3';
ledScale = 0.4;
ledMin = 0.30;
ledMax = 0.90;

ledDimmerLast = -0.75;
floodControlScale = 0;
floodButtonLast = 0;
floodHighlight = 0;
camHomeButtonLast = 0;
camCmdIO.(floodLED) = ledMin;


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 % Setup Camera Pan Tilt%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[panTiltArm, panTiltParams] = setupPanTiltArm(robotFamily);

panTiltNames = panTiltParams.names;

cameraMotors = 1:2;

cameraRotOffset = 180;   % [deg]
RPY = [0 0 0];

armDoF = [1 2];
camVelScale = 1;
cameraHome = [pi/2 pi/2];

panTiltCmd = CommandStruct();
panTiltCmd.position = nan(1,2);
panTiltCmd.velocity = nan(1,2);

panTiltFbk = panTiltArm.group.getNextFeedbackFull();
%panTiltarm.group.setFeedbackFrequency(100);

% Send Gains
%HebiUtils.sendWithRetry(panTiltArm.group, 'gains', armGains);

%Create whole robot group
robotNames = [driveNames panTiltNames camNames];
robotGroup = HebiLookup.newGroupFromNames(robotFamily, robotNames);


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Setup Mobile Phone Input %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Button Mapping
leftPadLeftRight = 'a1';
leftPadUpDown = 'a2';
flipperSlider1 = 'a3';
flipperSlider2 = 'a4';
flipperSlider3 = 'a5';
flipperSlider4 = 'a6';
rightPadLeftRight = 'a7';
rightPadUpDown = 'a8';

resetPoseButton = 'b1';
LEDButton = 'b3';
camHome = 'b4';
ledSlider = 'a3';
camHomeMode = 'b6';
quitDemoButton = 'b8';
% rotVelAxis = 'a1'; % Left Pad Left/Right

phoneName = 'mobileIO';
while true
    try
        fprintf('Searching for phone Controller...\n');
        phoneGroup = HebiLookup.newGroupFromNames(robotFamily, phoneName);
        phoneGroupIO = HebiMobileIO.findDevice(robotFamily, phoneName);
        disp('Phone Found.  Starting up');
        break;
    catch
        pause(0.5);
    end
    
    % Alternate Cyan and Magenta LEDs to signal "Searching for Controller"
    robotGroup.send('led','c');
    pause(1.0);
    robotGroup.send('led','m');
end



%%
%%%%%%%%%%%%%%%%%%%%%%%
 % Begin the demo loop %
%%%%%%%%%%%%%%%%%%%%%%%
    
% Start background logging
if enableLogging
    driveGroup.startLog('dir','logs');
    panTiltArm.group.startLog('dir','logs');
end
    

abortFlag = false;

while ~abortFlag
phoneCmd.f5 = ledDimmerLast;
phoneCmd.e1 = 1;  % Highlight robot reset button
phoneCmd.e8 = 1;  % Highlight Quit button
phoneCmd.e2 = 1;  % Highlight B2 button
phoneCmd.e4 = 1;  % Highlight B4 button
phoneCmd.e3 = floodHighlight; % Highlight B3 button on/off
phoneCmd.(camHomeMode) = 0;  % Momentary button
phoneCmd.(LEDButton) = 0;
controllerColor = 'b';

setupControllerLayout(phoneGroupIO);

HebiUtils.sendWithRetry(phoneGroup, ...
    'led', controllerColor, ... % send LED command
    'IoCommand', phoneCmd); % send button and slider settings

phoneGroup.send('clearLog',true);
phoneGroup.send('appendLog','Robot Homing Sequence');
phoneGroup.send('appendLog','Please wait...');

% LEDs to Magenta to signal "Homing Procedure"
robotGroup.send('led','m');

% Get initial feedback from entire robot
robotFbk = robotGroup.getNextFeedbackFull();

% Grab initial pose from phone
fbkPhoneMobile = phoneGroup.getNextFeedbackMobile();
fbkPhoneIO = phoneGroup.getNextFeedbackIO();

phoneFbkCounter = 0;
controllerLost = false;
controllerTextSent = false;
mStopPressed = false;
mStopTextSent = false;

% Initialize a trajectory for the base. (By passing velocities
% as positions we can compute a velocity trajectory. The constant
% time vector makes it act like a 'minimum-jerk' lowpass on the
% input)
loopTimer = tic();
timeNow = toc(loopTimer);
chassisTrajStartTime = timeNow;

velocities = zeros(2, 2);
chassisTraj = chassisTrajGen.newJointMove( velocities, ...
    'Time', [0 chassisParams.rampTime]);

% Initialize wheel position to current feedback and then
% integrate from there.
driveFbk = driveGroup.getNextFeedback();
driveCmd.position = driveFbk.position;

% Slow trajectory timing for the initial move to home position
panTiltArm.trajGen.setSpeedFactor( 0.5 );
panTiltArm.trajGen.setMinDuration( 1.5 );

% Move to arm home position
panTiltArm.update();
panTiltArm.setGoal(cameraHome);
while ~panTiltArm.isAtGoal()
    panTiltArm.update();
    panTiltArm.send();
end

% Reset behavior to normal speed
panTiltArm.trajGen.setSpeedFactor(0.9);
panTiltArm.trajGen.setMinDuration(0.33);
panTiltArm.group.setCommandLifetime(5);

camFbk = camGroup.getNextFeedbackFull();


% Get arm position feedback from camera
q_armBase = [ camFbk.orientationW(1) ...
              camFbk.orientationX(1) ...
              camFbk.orientationY(1) ...
              camFbk.orientationZ(1) ];
T_armBase = eye(4);
R_armBase = HebiUtils.quat2rotMat( q_armBase );
T_armBase(1:3,1:3) = R_armBase;
camKin.setBaseFrame( T_armBase );

% Set Mobile IO To "Control Mode" Screen Setup
    controllerColor = 'g';
    HebiUtils.sendWithRetry(phoneGroup, ...
        'led', controllerColor, ... % send LED command
        'IoCommand', phoneCmd); % send button and slider settings
    
    robotGroup.send('led',[]);
    
    phoneGroup.send('clearLog',true);
    
    while ~abortFlag
        
        %%%%%%%%%%%%%%%%%%%
        % Gather Feedback %
        %%%%%%%%%%%%%%%%%%%
        hasNewPhoneFbk = ~isempty(phoneGroup.getNextFeedback( ...
            fbkPhoneIO, fbkPhoneMobile, ... % overwrite existing structs
            'timeout', 0 )); % prevent blocking due to bad comms
        
        hasNewRobotFbk = ~isempty(robotGroup.getNextFeedback( ...
            robotFbk, 'View', 'Full', 'timeout', 0 ));
        
        if ~hasNewPhoneFbk
            if phoneFbkCounter < 20
                phoneFbkCounter = phoneFbkCounter + 1;
                pause(0.01);
            else
                controllerLost = true;
                pause(0.01);
                leftPadLeftRight = 0;
                leftPadUpDown = 0;
                rightPadLeftRight = 0;
                rightPadUpDown = 0;
                
            end
        else
            phoneFbkCounter = 0;
            controllerLost = false;
        end
        
        
       
        
        if ~hasNewRobotFbk
            pause(0.01);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%
        % Camera Orientation %
        %%%%%%%%%%%%%%%%%%%%%%
        camFbk = camGroup.getNextFeedbackFull();
        qZoom = [ camFbk.orientationW ...
            camFbk.orientationX ...
            camFbk.orientationY ...
            camFbk.orientationZ ];
        R_zoom = HebiUtils.quat2rotMat( qZoom );
        
        try
            RPY_old = RPY;
            RPY = SpinCalc( 'DCMtoEA313', R_zoom, 1E6, 0 );
        catch
            disp('Yo!');
            RPY = RPY_old;
        end
        
        cameraRoll_imu = RPY(1) - cameraRotOffset;
        if ~exist('cameraRoll_gyro','var')
            cameraRoll_gyro = cameraRoll_imu;
        else
            cameraRoll_gyro = cameraRoll + camFbk.gyroZ(1)*dt;
        end
        
        pitchCheck = RPY(1);
        
        % Singularity Check - Video Rotation
        deadZoneRange = 10;
        smoothZoneRange = 40;
        alphaRange = smoothZoneRange - deadZoneRange;
        if pitchCheck < deadZoneRange || pitchCheck > 180-deadZoneRange
            cameraRoll = cameraRoll_gyro;
        elseif pitchCheck <= smoothZoneRange || pitchCheck >= 180-smoothZoneRange
            if pitchCheck > 90
                pitchCheck = 180 - pitchCheck;
            end
            alpha = (abs(pitchCheck - deadZoneRange) / alphaRange);
            cameraRoll = alpha*cameraRoll_imu + (1-alpha)*cameraRoll_gyro;
            cameraRoll_imu = cameraRoll;
        else
            cameraRoll = cameraRoll_imu;
            cameraRoll_gyro = cameraRoll_imu;
        end
        
        R_cam = R_z( deg2rad( cameraRoll ) );
        
        phoneCmd.('c1') = [deg2rad(cameraRoll)];
        phoneGroup.send(phoneCmd);
        
        
        
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
        
        
        %%%%%%%%%%%%%%%%%%%%%%%
        % Mobile Base Control %
        %%%%%%%%%%%%%%%%%%%%%%%
        % Map joystick input to linear speeds
        xVelAxis = rightPadUpDown;
        yVelAxis = rightPadLeftRight;
        xVel = maxLinSpeed * fbkPhoneIO.(xVelAxis);
        yVel = maxLinSpeed * fbkPhoneIO.(yVelAxis);
        desiredChassisVel = [xVel yVel];
        
        % Find the current point in chassis trajectory
        % (linear velocity) we plan a smooth transition
        t = min(timeNow - chassisTrajStartTime, chassisTraj.getDuration);
        [chassisVel, chassisAccel, chassisJerk] = chassisTraj.getState(t);
        
        % Compute a trajectory that smoothly transitions to
        % the new desired velocities
        velocities = [chassisVel; desiredChassisVel];
        accelerations = [chassisAccel; zeros(1,2) ];
        jerks = [chassisJerk; zeros(1,2) ];
        chassisTraj = chassisTrajGen.newJointMove( velocities, ...
            'Velocities', accelerations, ...
            'Accelerations', jerks, ...
            'Time', [0 chassisParams.rampTime] );
        chassisTrajStartTime = timeNow;
        
        % Compute dt since the last update for integrating position
        timeLast = timeNow;
        timeNow = toc(loopTimer);
        dt = timeNow - timeLast;
        
        % Convert linear velocities into wheel/joint velocities
        driveCmd.velocity = (chassisToWheelVelocities * chassisVel')';
        driveCmd.position = driveCmd.position + driveCmd.velocity * dt;
        driveCmd.effort = (chassisEffortsToWheelEfforts * ...
            (chassisMassMatrix * chassisAccel'))';
        driveGroup.send(driveCmd);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%
        % Camera / LED Controls %
        %%%%%%%%%%%%%%%%%%%%%%%%%
        
        ledDimmer = (fbkPhoneIO.(ledSlider) + 1.0) / 2;
        if ledDimmer < ledMin
            ledDimmer = 0;
        elseif ledDimmer > ledMax
            ledDimmer = ledMax;
        end

        
        if (fbkPhoneIO.(LEDButton) - floodButtonLast) == 1
            floodControlScale = ~floodControlScale;
            floodHighlight = ~floodHighlight;
            ledBtnCmd.e3 = floodHighlight;  % Highlight B3 button on/off
            HebiUtils.sendWithRetry(phoneGroup, ...
                    'IoCommand', camCmdIO); % send button and slider settings
        end
        
        floodButtonLast = fbkPhoneIO.(LEDButton);
        
        floodCmd = floodControlScale * ledDimmer;
      
        camCmdIO.(floodLED) = floodCmd;
        camCmdIO.(floodLED) = min(camCmdIO.(floodLED), ledMax );
        
        
        phoneGroup.send(camCmdIO);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Camera Pan Tilt Base Control %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        camTiltJoy = leftPadUpDown;
        camPanJoy = leftPadLeftRight;
        camPan = -sign(fbkPhoneIO.(camPanJoy)) * fbkPhoneIO.(camPanJoy)^2;
        camTilt = sign(fbkPhoneIO.(camTiltJoy)) * fbkPhoneIO.(camTiltJoy)^2;
        panVel = camVelScale * camPan;
        tiltVel = camVelScale * camTilt;
        
        
        fbkPanTiltPos = panTiltFbk.position(armDoF);
        if any(isnan(panTiltFbk.positionCmd))
            cmdPanTiltPos = fbkPanTiltPos;
        else
            cmdPanTiltPos = panTiltFbk.position(armDoF);
        end
        
        % Differential IK for camera velocity control
        T_fbk = camKin.getFK( 'endeffector', fbkPanTiltPos );
        R_fbk = T_fbk(1:3,1:3);
        T_cmd = camKin.getFK( 'endeffector', cmdPanTiltPos );
        R_cmd = T_cmd(1:2,1:2);
        
        % Get the Jacobian for just the rotational components of the camera,
        % and just the 2 wrist actuators.
        J_fbk = camKin.getJacobian( 'endeffector', fbkPanTiltPos  );
        J_cam = J_fbk(4:6,1:2);  % This is in the base frame of the robot
        J_inv_cam = pinv_damped( J_cam );
        
        % Rotate the desired camera velocites to align with gravity / camera
        % rotation and put them in the base frame.
        cameraMotors = 1:2;
        camRotVelVec = [panVel; tiltVel; 0];
        
        camRotVelVec = R_fbk * R_cam * camRotVelVec;
%         camRotVelVec = R_fbk * camRotVelVec;
        camVels = J_inv_cam * camRotVelVec;
%         camVels = camRotVelVec;
        
        if (fbkPhoneIO.(camHome) - camHomeButtonLast) == 1
        end

        % Camera commands
        panTiltCmd.velocity(cameraMotors) = camVels;
        panTiltCmd.position(cameraMotors) = panTiltCmd.position(cameraMotors) + ...
        panTiltCmd.velocity(cameraMotors)*dt;
        
        panTiltArm.group.send(panTiltCmd());
        
        
        
    end


end

    
    if enableLogging
        driveGroup.stopLogFull();
    end

end