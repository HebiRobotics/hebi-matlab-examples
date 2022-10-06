% Mobile Base with 6-DoF arm & gripper.  Tele-op using HEBI Mobile I/O App.
%
% Dave Rollinson
% July 2018

%%
function mobileBaseChevron( mobileBaseType )

HebiLookup.initialize();

% Optional step to limit the lookup to a set of interfaces or modules
% HebiLookup.setLookupAddresses('10.10.10.255');
enableLogging = true;
robotFamily = 'Chevron';

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

% Create Group
wheelGroup = HebiLookup.newGroupFromNames(robotFamily, chassisParams.wheelModuleNames);
HebiUtils.sendWithRetry(wheelGroup, 'gains', chassisParams.wheelGains);
wheelCmd = CommandStruct();

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % Setup Camera %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

camGroup = HebiLookup.newGroupFromNames( robotFamily, 'Widey' );
camCmdIO = IoCommandStruct();
panTiltGroup = HebiLookup.newGroupFromNames(robotFamily, ...
                {'C1_pan', 'C2_tilt'});


floodLED = 'f3';
ledScale = 0.4;
ledMin = 0.30;
ledMax = 0.90;

ledDimmerLast = -0.75;
floodControlScale = 0;
floodButtonLast = 0;
floodHighlight = 0;

camCmdIO.(floodLED) = ledMin;

cameraRotOffset = 0;   % [deg]
cameraHome = [pi/2 pi/2];
RPY = [0 0 0];

camKin = HebiKinematics('hrdf/Chevron-CamTail');
armDoF = [1 2];

panTiltCmd = CommandStruct();
panTiltCmd.position = nan(1,2);
panTiltCmd.velocity = nan(1,2);

panTiltFbk = panTiltGroup.getNextFeedbackFull();

%Get initial camera pose
fbkPanTiltPos = panTiltFbk.position(armDoF);

q_armBase = [ panTiltFbk.orientationW(1) ...
  panTiltFbk.orientationX(1) ...
  panTiltFbk.orientationY(1) ...
  panTiltFbk.orientationZ(1) ];

R_armBase = HebiUtils.quat2rotMat( q_armBase );

T_armBase = eye(4);
T_armBase(1:3,1:3) = R_armBase;
camKin.setBaseFrame( T_armBase );

T_fbk = camKin.getFK( 'endeffector', fbkPanTiltPos );
R_fbk = T_fbk(1:3,1:3);
rotMatTarget = R_fbk;
R_fbk_init = R_fbk;

%% Camera Streams Setup



%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Setup Mobile Phone Input %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Button Mapping
flipperSlider1 = 'a3';
flipperSlider2 = 'a4';
flipperSlider3 = 'a5';
flipperSlider4 = 'a6';
resetPoseButton = 'b1';
rightPadUpDown = 'a8';
rightPadLeftRight = 'a7';
leftPadUpDown = 'a2';
leftPadLeftRight = 'a1';
LEDButton = 'b3';
camHome = 'b4';
ledSlider = 'a3';
camHomeMode = 'b6';
quitDemoButton = 'b8';
probeXAxis = 'a1'; % Right Pad Up/Down
probeYAxis = 'a2'; % Right Pad Left/Right
xVelAxis = 'a8'; % Right Pad Up/Down
yVelAxis = 'a7'; % Right Pad Left/Right
% rotVelAxis = 'a1'; % Left Pad Left/Right


camVelScale = 1; % rad/sec

% Search for phone controller. Allow retry because phones tend to
% drop out when they aren't used (i.e. sleep mode)
phoneName = 'mobileIO';
while true
    try
        fprintf('Searching for phone Controller...\n');
        phoneGroup = HebiLookup.newGroupFromNames(robotFamily, phoneName);
        phoneGroupIO = HebiMobileIO.findDevice(robotFamily, phoneName);
        disp('Phone Found.  Starting up');
        break;
    catch
        pause(1.0);
    end
end

% % IO Command Setup - Controller Layout
%     phoneGroupIO.initializeUI();
%     unicode = @(input) sprintf(strrep(input, '\u', '\x'));
%     
%     phoneGroupIO.setAxisSnap([3 5], [0 0]);
%     phoneGroupIO.setAxisValue(6, -1);
%     phoneGroupIO.setAxisLabel([3 4 5 6 7 8], {unicode('\u21D5'),unicode('\uD83D\uDD26'), ...
%         'LED','Zm','Y','X'});
%     phoneGroupIO.setButtonIndicator([1 2 3 4 5 6 7 8], [true true true true true true true true]);
%     phoneGroupIO.setButtonLabel([1 2 3 4 5 6 7 8], {unicode('\u21BA'), 'T', 'Spot', 'Fld', 'fwd','Home', 'rear', ...
%         unicode('\u23F9')});
%     phoneGroupIO.clearText();

%%
%%%%%%%%%%%%%%%%%%%%%%%
% Begin the demo loop %
%%%%%%%%%%%%%%%%%%%%%%%

% Start background logging
if enableLogging
    %         arm.group.startLog('dir','logs');
    wheelGroup.startLog('dir','/chassis');
end

% This outer loop is what we fall back to anytime we 're-home' the arm
loopTimer = tic();
timeNow = toc(loopTimer);

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
    
       
%     HebiUtils.sendWithRetry(phoneGroup, ...
%         'led', controllerColor, ... % send LED command
%         'IoCommand', phoneCmd); % send button and slider settings
    
    % IO Command Setup - Controller Layout
    phoneGroupIO.initializeUI();
    unicode = @(input) sprintf(strrep(input, '\u', '\x'));
    
    phoneGroupIO.setAxisSnap([3 5], [0 0]);
    phoneGroupIO.setAxisValue(6, -1);
    phoneGroupIO.setAxisLabel([3], {unicode('\uD83D\uDD26')});
    phoneGroupIO.setButtonIndicator([1 2 3 4 5 6 7 8], [false false true false false false false true]);
    phoneGroupIO.setButtonLabel([3 7 8], {'Fld', 'Home', unicode('\u23F9')});
    phoneGroupIO.clearText();
    
    %Grab initial pose from phone
    [fbkPhoneMobile, fbkPhoneIO] = phoneGroupIO.getFeedback();
    
    phoneFbkCounter = 0;
    controllerLost = false;
    controllerTextSent = false;
    mStopPressed = false;
    mStopTextSent = false;

    
    % Initialize a trajectory for the base. (By passing velocities
    % as positions we can compute a velocity trajectory. The constant
    % time vector makes it act like a 'minimum-jerk' lowpass on the
    % input)
    timeNow = toc(loopTimer);
    chassisTrajStartTime = timeNow;
    
    velocities = zeros(2, 2);
    chassisTraj = chassisTrajGen.newJointMove( velocities, ...
        'Time', [0 chassisParams.rampTime]);
    
    % Initialize wheel position to current feedback and then
    % integrate from there.
    wheelFbk = wheelGroup.getNextFeedback();
    wheelCmd.position = wheelFbk.position;
    
    camVelScale = 1;
    q_armBase = [ panTiltFbk.orientationW(1) ...
                  panTiltFbk.orientationX(1) ...
                  panTiltFbk.orientationY(1) ...
                  panTiltFbk.orientationZ(1) ];
    T_armBase = eye(4);
    R_armBase = HebiUtils.quat2rotMat( q_armBase );
    T_armBase(1:3,1:3) = R_armBase;
    camKin.setBaseFrame( T_armBase );
    
    phoneCmdCam = IoCommandStruct();
    
    while ~abortFlag
         %%%%%%%%%%%%%%%%%%%
        % Gather Feedback %
        %%%%%%%%%%%%%%%%%%%
        
        
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
        
        % If you lose connection to the iPad, stop all velocity commands
        if ~hasNewPhoneFbk
            if phoneFbkCounter < 20
                phoneFbkCounter = phoneFbkCounter + 1;
                pause(0.01);
            else
                controllerLost = true;
                pause(0.01);
                probeXAxis = 0; % Right Pad Up/Down
                probeYAxis = 0; % Right Pad Left/Right
                xVelAxis = 0; % Right Pad Up/Down
                yVelAxis = 0;
            end
        else
            phoneFbkCounter = 0;
            controllerLost = false;
        end
        
        if controllerLost && ~controllerTextSent
            disp('Lost connection to Controller. Please reconnect.')
%             robotGroup.send('led','b');
%             arm.group.send('led','b');
            controllerTextSent = true;
        end
        
        if ~controllerLost && controllerTextSent
            disp('Controller reconnected, demo continued.')
            phoneGroup.clearText();
           controllerTextSent = false;
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
        
        phoneCmdCam.('c1') = [deg2rad(cameraRoll)];
        phoneGroup.send(phoneCmdCam);
        
        
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
        
        
        camGroup.send(camCmdIO);

        % If you lose connection to the iPad, stop all velocity commands
    if ~hasNewPhoneFbk
        if phoneFbkCounter < 20
            phoneFbkCounter = phoneFbkCounter + 1;
            pause(0.01);
        else
            flip = 0;
%             tiltJoyVel = 0;
%             panJoyVel = 0;
            controllerLost = true;
            pause(0.01);
            probeXVel = 0;
            probeYVel = 0;
        end
    else
        phoneFbkCounter = 0;
        controllerLost = false;
    end
    
    if controllerLost && ~controllerTextSent
        disp('Lost connection to Controller. Please reconnect.')
        robotGroup.send('led','b');
        controllerTextSent = true;
    end
    
    if ~controllerLost && controllerTextSent
        disp('Controller reconnected, demo continued.')
        
        phoneGroup.setAxisSnap([3 5], [0 0]);
        phoneGroup.setAxisValue(6, -1);
        phoneGroup.setAxisLabel([3 4 5 6 7 8], {unicode('\u21D5'),unicode('\uD83D\uDD26'), ...
            'LED','Zm'});
        phoneGroup.setButtonIndicator([1 2 3 4 5 6 7 8], [true true true true true true true true]);
        phoneGroup.setButtonLabel([1 2 3 4 5 6 7 8], {unicode('\u21BA'), 'T', 'Spot', 'Fld', 'fwd','Home', 'rear', ...
            unicode('\u23F9')});
        phoneGroup.clearText();
        
        robotGroup.send('led',[]);
        controllerTextSent = false;
    end
        
        %%%%%%%%%%%%%%%%%%%%%%%
        % Mobile Base Control %
        %%%%%%%%%%%%%%%%%%%%%%%
        % Map joystick input to linear speeds
        xVel = maxLinSpeed * fbkPhoneIO.(xVelAxis);
        yVel = maxLinSpeed * fbkPhoneIO.(yVelAxis);
%         rotVel = maxRotSpeed * fbkPhoneIO.(rotVelAxis);
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
        wheelCmd.velocity = (chassisToWheelVelocities * chassisVel')';
        wheelCmd.position = wheelCmd.position + wheelCmd.velocity * dt;
        wheelCmd.effort = (chassisEffortsToWheelEfforts * ...
            (chassisMassMatrix * chassisAccel'))';
        wheelGroup.send(wheelCmd);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Camera Pan Tilt Base Control %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        camTiltJoy = leftPadUpDown;
        camPanJoy = leftPadLeftRight;
        camPan = -sign(fbkPhoneIO.(camPanJoy)) * fbkPhoneIO.(camPanJoy)^2;
        camTilt = -sign(fbkPhoneIO.(camTiltJoy)) * fbkPhoneIO.(camTiltJoy)^2;
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
        
        panTiltGroup.send(panTiltCmd());
        
    end
end

if enableLogging
    wheelGroup.stopLogFull();

end
