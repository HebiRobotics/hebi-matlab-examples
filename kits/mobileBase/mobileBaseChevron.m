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
%     %%%%%%%%%%%%%%%%%%%%%%%%%
%     % Setup Arm and Gripper %
%     %%%%%%%%%%%%%%%%%%%%%%%%%
%
%     [ arm, armParams, gripper ] = setupArmWithGripper(robotFamily);
%
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
panTiltGroup = HebiLookup.newGroupFromNames(robotFamily, ...
                {'C1_pan', 'C2_tilt'});
camCmdIO = IoCommandStruct();

LED = 'f1';
ledScale = 0.4;
ledMin = 0.30;
ledMax = 0.90;

ledDimmerLast = -0.75;
ledControlScale = 0;
ledButtonLast = 0;
ledHighlight = 0;
camHomeButtonLast = 0;

camCmdIO.(LED) = ledMin;

cameraRotOffset = 0;   % [deg]
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
LEDButton = 'b3';
LEDSlider = 'a3';
camHomeMode = 'b6';
quitDemoButton = 'b8';
probeXAxis = 'a1'; % Right Pad Up/Down
probeYAxis = 'a2'; % Right Pad Left/Right
xVelAxis = 'a8'; % Right Pad Up/Down
yVelAxis = 'a7'; % Right Pad Left/Right
% rotVelAxis = 'a1'; % Left Pad Left/Right

phoneControlScale = 0;
camVelScale = 1; % rad/sec

% Search for phone controller. Allow retry because phones tend to
% drop out when they aren't used (i.e. sleep mode)
phoneName = 'mobileIO';
while true
    try
        fprintf('Searching for phone Controller...\n');
        phoneGroup = HebiLookup.newGroupFromNames(robotFamily, phoneName);
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
    %         arm.group.startLog('dir','logs');
    wheelGroup.startLog('dir','logs');
end

% This outer loop is what we fall back to anytime we 're-home' the arm
loopTimer = tic();
timeNow = toc(loopTimer);
abortFlag = false;

while ~abortFlag
    phoneCmd.(flipperSlider1) = 0; % Snap slider to center
    phoneCmd.(flipperSlider2) = 0; 
    phoneCmd.(flipperSlider3) = 0; 
    phoneCmd.(flipperSlider4) = 0; 
    phoneCmd.f5 = ledDimmerLast;
    phoneCmd.e1 = 1;  % Highlight robot reset button
    phoneCmd.e8 = 1;  % Highlight Quit button
    phoneCmd.e2 = 1;  % Highlight B2 button
    phoneCmd.e4 = 1;  % Highlight B4 button
    phoneCmd.e3 = 1;  % Highlight B3 button
    phoneCmd.e5 = ledHighlight;
    phoneCmd.(camHomeMode) = 0;  % Momentary button
    controllerColor = 'b';
    
    
    % Grab initial pose from phone
    fbkPhoneMobile = phoneGroup.getNextFeedbackMobile();
    panTiltFbk = panTiltGroup.getNextFeedbackFull();
    fbkPhoneIO = phoneGroup.getNextFeedbackIO();    
    
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
        
        pitchCheck = RPY(2);
        
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
        camTiltJoy = probeYAxis;
        camPanJoy = probeXAxis;
        camPan = sign(fbkPhoneIO.(camPanJoy)) * fbkPhoneIO.(camPanJoy)^2;
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
        

        % Camera commands
        panTiltCmd.velocity(cameraMotors) = camVels;
        panTiltCmd.position(cameraMotors) = panTiltCmd.position(cameraMotors) + ...
            panTiltCmd.velocity(cameraMotors)*dt;
        
        panTiltGroup.send(panTiltCmd());
        camGroup.send(camCmdIO);
        
    end
end

if enableLogging
    wheelGroup.stopLogFull();
end
end
