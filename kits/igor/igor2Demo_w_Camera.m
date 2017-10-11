% Testing out balancing robot control.
%
% Assumes using a Sony PS4 Gamepad:
% Model CUH-ZCT2U Wireless Controller
%
% Dave Rollinson
% Apr 2017

clear *;
close all;

moduleNames = { 'wheel1', 'wheel2', ...
                'hip1', 'knee1', ...
                'hip2', 'knee2', ...
                'base1', 'shoulder1', 'elbow1', 'wrist1', ...
                'base2', 'shoulder2', 'elbow2', 'wrist2', ...
                'camTilt'};

numDOFs = 15;  
wheelDOFs = [1,2];
legDOFs{1} = 3:4;
legDOFs{2} = 5:6;
armDOFs{1} = 7:10;
armDOFs{2} = 11:14;
            
robotGroup = HebiLookup.newGroupFromNames('Igor II',moduleNames);
robotGroup.setFeedbackFrequency(300);

pause(1.0);

gains = HebiUtils.loadGains('igorGains');
robotGroup.send('gains',gains);

fbk = robotGroup.getNextFeedbackFull();
timeLast = fbk.time;

animStruct = struct();

joy = HebiJoystick(1);
[axes, buttons, povs] = read(joy);
axesLast = axes;

%Workaround for bug that makes the triggers not work correctly until they
%are both pushed down fully for the first time.
while(axes(4)~=-1.000 || axes(5)~=-1.000)
    [axes, buttons, povs] = read(joy);
    robotGroup.send('led','b');
    pause(0.1);
    robotGroup.send('led','m');
    pause(0.1)
end

robotGroup.send('led',[]);

balanceOn = true;
logging = false;   % Flag to turn logging on and off.  If logging is on you 
                  % you can view a bunch of debug plots after quitting.
  
cmd = CommandStruct();
cmd.position = nan(1,numDOFs);
cmd.velocity = nan(1,numDOFs);
cmd.effort = nan(1,numDOFs);

direction = [1, -1];

wheelRadius = .200 / 2;  % m
wheelBase = .43;  % m

chassisCoM = [0; 0; .10 + .3];  % m
chassisMass = 6;  % kg 9

numLegs = 2;
numArms = 2;

J_limit = .010;

gripVel = zeros(3,numArms);
wristVel = 0;
hipPitchVel = 0;

camTiltVel = 0;
camTiltPos = 0;

hipPosComp = -.0;
hipPitch = 0;

%imuModules = [1:6,7,10];
imuModules = 1:6;

R_hip1 = R_x(-pi/2);
R_hip2 = R_x(pi/2);

leanAngleErrorCum = 0;
chassisVelErrorCum = 0;
fbkChassisVelLast = 0;
cmdChassisVelLast = 0;

timeHist = nan(0,1);
cmdLeanAngleHist = nan(0,1);
fbkLeanAngleHist = nan(0,1);
leanAngleVelHist = nan(0,1);
leanAngleOffsetHist = nan(0,1);
cmdChassisVelHist = nan(0,1);
fbkChassisVelHist = nan(0,1);
RPYHist = nan(0,3);

%%%%%%%%%%%%%%%%%%
% Leg Kinematics %
%%%%%%%%%%%%%%%%%%

kneeAngle = deg2rad(130);
hipAngle = pi/2 + kneeAngle/2;
legHomeAngles = [ hipAngle  kneeAngle;
                 -hipAngle -kneeAngle ];

% Setup Legs
R_hip = R_x(pi/2);
xyz_hip = [0; .0225; .055]; 
T_hip = eye(4);
T_hip(1:3,1:3) = R_hip;
T_hip(1:3,4) = xyz_hip;

legBaseFrames(:,:,1) = eye(4);
legBaseFrames(:,:,2) = eye(4);

legBaseFrames(1:3,4,1) = [0; .15; 0];
legBaseFrames(1:3,4,2) = [0; -.15; 0];

legBaseFrames(1:3,1:3,1) = R_x(-pi/2);
legBaseFrames(1:3,1:3,2) = R_x(pi/2);

for leg = 1:numLegs
    legKin{leg} = HebiKinematics();
    legKin{leg}.addBody('X5-9');
    legKin{leg}.addBody('X5Link','ext',.375,'twist',pi);
    legKin{leg}.addBody('X5-4');
    legKin{leg}.addBody('X5Link','ext',.325,'twist',pi);

    legKin{leg}.setBaseFrame(legBaseFrames(:,:,leg));
end

%%%%%%%%%%%%%%%%%%
% Arm Kinematics %
%%%%%%%%%%%%%%%%%%

armBaseXYZ(:,1) = [0; .10; .20];
armBaseXYZ(:,2) = [0; -.10; .20];

mounting = {'left-inside','right-inside'};

for arm = 1:numArms
    armKin{arm} = HebiKinematics();
    armKin{arm}.addBody('X5-4');
    armKin{arm}.addBody('X5-HeavyBracket', 'mount', mounting{arm} );
    armKin{arm}.addBody('X5-9');
    armKin{arm}.addBody('X5Link','ext',.325,'twist',0,'mass',.250);
    armKin{arm}.addBody('X5-4');
    armKin{arm}.addBody('X5Link','ext',.325,'twist',pi,'mass',.350);
    armKin{arm}.addBody('X5-4');

    armTransform = eye(4);
    armTransform(1:3,4) = armBaseXYZ(:,arm);
    armKin{arm}.setBaseFrame(armTransform);
end

armHomeAngles(1,:) = deg2rad([0 20 60 0]);
armHomeAngles(2,:) = deg2rad([0 -20 -60 0]);

armJointAngs = armHomeAngles';
armJointVels = zeros(size(armJointAngs));
for arm = 1:numArms
    T_endEffector = armKin{arm}.getFK('endeffector',armHomeAngles(arm,:));
    gripPos(:,arm) = T_endEffector(1:3,4);
end

gripPos(2,2) = -gripPos(2,2);
gripPos = repmat(mean(gripPos,2),1,2);
gripPos(2,2) = -gripPos(2,2);

if logging
    robotGroup.startLog();
end

%%%%%%%%%%%%%%%%
% Soft Startup %
%%%%%%%%%%%%%%%%
startupTime = 3.0;
time = [ 0 startupTime ];
   
for i = 1:2
    
    % Leg Trajectory Generator
    trajGenLegs{i} = HebiTrajectoryGenerator(legKin{i});
    trajGenLegs{i}.setSpeedFactor(1);
    trajGenLegs{i}.setAlgorithm('UnconstrainedQp');

    pos = [ fbk.position(legDOFs{i}); 
            legHomeAngles(i,:) ];
    vel = zeros(2,2);
    accel = zeros(2,2);

    legTraj{i} = trajGenLegs{i}.newJointMove( pos, ...
                'Velocities', vel, ...
                'Accelerations', accel, ...
                'Time', time );
            
    % Arm Trajectory Generator
    trajGenArms{i} = HebiTrajectoryGenerator(armKin{i});
    trajGenArms{i}.setSpeedFactor(1);
    trajGenArms{i}.setAlgorithm('UnconstrainedQp');

    pos = [ fbk.position(armDOFs{i}); 
            armHomeAngles(i,:) ];
    vel = zeros(2,4);
    accel = zeros(2,4);

    armTraj{i} = trajGenArms{i}.newJointMove( pos, ...
                'Velocities', vel, ...
                'Accelerations', accel, ...
                'Time', time );        
end   

% Initialize Pose Filter
poseFilter = HebiPoseFilter();
poseFilter.setMaxAccelWeight( .01);
poseFilter.setMaxAccelNormDev( .3 );
filterTime = fbk.time;

trajTimer = tic;       
t = toc(trajTimer);

while t < startupTime

    t = toc(trajTimer);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Update date filtered pose estimate %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    try
        fbk = robotGroup.getNextFeedback( fbk );
    catch
        disp('Could not get feedback!');
        break;
    end
    dt = mean(fbk.time - timeLast);
    dt = min(dt,.02);
    timeLast = fbk.time;
    filterTime = filterTime + dt;
    
    legFK{1} = legKin{1}.getFK('output',fbk.position(legDOFs{1}));
    legFK{2} = legKin{2}.getFK('output',fbk.position(legDOFs{2}));

    accel = [fbk.accelX; fbk.accelY; fbk.accelZ];
    gyro = [fbk.gyroX; fbk.gyroY; fbk.gyroZ];

    poseAccel = accel(:,imuModules);
    poseGyro = gyro(:,imuModules);

    imuFrames(:,:,1) = legFK{1}(:,:,4);
    imuFrames(:,:,2) = legFK{2}(:,:,4);
    imuFrames(:,:,3) = legKin{1}.getBaseFrame;
    imuFrames(:,:,4) = legFK{1}(:,:,2);
    imuFrames(:,:,5) = legKin{2}.getBaseFrame;
    imuFrames(:,:,6) = legFK{2}(:,:,2);
    imuFrames(:,:,7) = armKin{1}.getBaseFrame;
    imuFrames(:,:,8) = armKin{1}.getBaseFrame;

    rotComp = zeros(3,8);

    for j=1:length(imuModules)
        poseAccel(:,j) = imuFrames(1:3,1:3,j) * poseAccel(:,j);
        poseGyro(:,j) = imuFrames(1:3,1:3,j) * (poseGyro(:,j) + rotComp(:,j));
    end

    poseAccel;
    poseGyro;

    poseAccelMean = mean(poseAccel,2);
    poseGyroMean = mean(poseGyro,2);

    poseFilter.update( poseAccelMean, poseGyroMean, filterTime );   
    T_pose = poseFilter.getPose();

    % Limit commands initially
    softStart = min(t/startupTime,1);
    
    for i=1:2
        [posNow, velNow, accNow] = legTraj{i}.getState(t);
        cmd.position(legDOFs{i}) = posNow;
        cmd.velocity(legDOFs{i}) = velNow;
        
        [posNow, velNow, accNow] = armTraj{i}.getState(t);
        cmd.position(armDOFs{i}) = posNow;
        cmd.velocity(armDOFs{i}) = velNow;
        
        % Gravity Compensation Torques
        cmd.effort(armDOFs{i}) = armKin{i}.getGravCompEfforts( ...
                                fbk.position(armDOFs{i}), ...
                                -T_pose(3,1:3) );                     
    end
    
    cmd.effort = softStart * cmd.effort;   
    robotGroup.send(cmd);
end

warmupTimer = tic;
warmupDuration = 1;  % sec

while true
    
    % Limit commands initially
    softStart = min(toc(warmupTimer)/warmupDuration,1);
    
    try
        fbk = robotGroup.getNextFeedback( fbk );
    catch
        disp('Could not get feedback!');
        break;
    end
    dt = mean(fbk.time - timeLast);
    dt = min(dt,.005);
    timeLast = fbk.time;
    filterTime = filterTime + dt;
    
    %%%%%%%%%%%%%%%%%%%%%%
    % Get Leg Kinematics %
    %%%%%%%%%%%%%%%%%%%%%%
    for leg=1:numLegs
        legCoMs{leg} = legKin{leg}.getFK('com',fbk.position(legDOFs{leg}));
        legFK{leg} = legKin{leg}.getFK('output',fbk.position(legDOFs{leg}));
        legTipFK{leg} = legKin{leg}.getFK( 'endeffector', ...
                                fbk.position(legDOFs{leg}) );
        J_legFbk{leg} = legKin{leg}.getJacobianEndEffector( ...
                                fbk.position(legDOFs{leg}) );                       
%         det_J_fbk(leg) = abs(det(J_fbk{leg}(1:3,:)));        
        
        J_legCmd{leg} = legKin{leg}.getJacobianEndEffector( ...
                                fbk.positionCmd(legDOFs{leg}) );
%         det_J_cmd(leg) = abs(det(J_cmd{leg}(1:3,:))); 
        
        legXYZ = squeeze(legCoMs{leg}(1:3,4,:));
        legMasses = legKin{leg}.getBodyMasses;
        legCoM(:,leg) = sum( legXYZ.*repmat(legMasses',3,1), 2 ) / sum(legMasses);
        legMass(leg) = sum(legMasses);
    end
    
    %%%%%%%%%%%%%%%%%%%%%%
    % Get Arm Kinematics %
    %%%%%%%%%%%%%%%%%%%%%%
    for i=1:numArms
        armFK = armKin{i}.getFK('com',fbk.position(armDOFs{i}));
        armTipFK{i} = armKin{i}.getFK( 'endeffector', ...
                                fbk.position(armDOFs{i}) );
        J_armFbk{i} = armKin{i}.getJacobianEndEffector( ...
                                fbk.position(armDOFs{i}) );                       
        det_J_fbk(i) = abs(det(J_armFbk{i}(1:3,1:3)));        
        
        J_armCmd{i} = armKin{i}.getJacobianEndEffector( ...
                                fbk.positionCmd(armDOFs{i}) );
        det_J_cmd(i) = abs(det(J_armCmd{i}(1:3,1:3))); 
        
        armXYZ = squeeze(armFK(1:3,4,:));
        armMasses = armKin{i}.getBodyMasses;
        armCoM(:,i) = sum( armXYZ.*repmat(armMasses',3,1), 2 ) / sum(armMasses);
        armMass(i) = sum(armMasses);
    end
    
    % Adjust CoM based on chassis, leg configuration
    allCoMs = [legCoM armCoM chassisCoM];
    allMasses = [legMass armMass chassisMass];
    
    robotCoM = sum(allCoMs .* repmat(allMasses,3,1), 2) / sum(allMasses);
    robotMass = sum(allMasses);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Update date filtered pose estimate %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    accel = [fbk.accelX; fbk.accelY; fbk.accelZ];
    gyro = [fbk.gyroX; fbk.gyroY; fbk.gyroZ];
    
    poseAccel = accel(:,imuModules);
    poseGyro = gyro(:,imuModules);
    
    imuFrames(:,:,1) = legFK{1}(:,:,4);
    imuFrames(:,:,2) = legFK{2}(:,:,4);
    imuFrames(:,:,3) = legKin{1}.getBaseFrame;
    imuFrames(:,:,4) = legFK{1}(:,:,2);
    imuFrames(:,:,5) = legKin{2}.getBaseFrame;
    imuFrames(:,:,6) = legFK{2}(:,:,2);
    imuFrames(:,:,7) = armKin{1}.getBaseFrame;
    imuFrames(:,:,8) = armKin{2}.getBaseFrame;
    
    rotComp = zeros(3,8);
    
    for i=1:length(imuModules)
        poseAccel(:,i) = imuFrames(1:3,1:3,i) * poseAccel(:,i);
        poseGyro(:,i) = imuFrames(1:3,1:3,i) * (poseGyro(:,i) + rotComp(:,i));
    end
    
    poseAccel;
    poseGyro;
    
    poseAccelMean = mean(poseAccel,2);
    poseGyroMean = mean(poseGyro,2);
    
    poseFilter.update( poseAccelMean, poseGyroMean, filterTime ); 
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%
    % Calculate Lean Angle %
    %%%%%%%%%%%%%%%%%%%%%%%%  
    T_pose = poseFilter.getPose();
    
    RPY = SpinCalc('DCMtoEA123',T_pose(1:3,1:3),1E-9,0);
    RPY(RPY>180) =  RPY(RPY>180) - 360;
    
    rollAngle = RPY(1);
    pitchAngle = RPY(2);

    leanR = R_y(-deg2rad(pitchAngle));
    
    leanAngle = -pitchAngle;
    leanAngleVel = poseGyroMean(2);
%     
%     leanAngleOffset = 0;
%     fbkLeanAngle = leanAngle - leanAngleOffset;

    groundPoint = mean([legTipFK{1}(1:3,4),legTipFK{2}(1:3,4)],2);      
    lineCoM = leanR*(robotCoM - groundPoint);
    heightCoM = norm(lineCoM);
    
    fbkLeanAngle = rad2deg(atan2(lineCoM(1),lineCoM(3)));
    
    robotCoM = T_pose(1:3,1:3) * robotCoM;
    for leg=1:numLegs
        for i=1:size(legCoMs{leg},3)
            legCoMs{leg}(:,:,i) = T_pose * legCoMs{leg}(:,:,i);
        end
        legTipLeanFK{leg} = T_pose * legTipFK{leg};
    end
    
%     plotFrames = cat(3,T_pose,legCoMs{1},legCoMs{2},legTipLeanFK{1},legTipLeanFK{2});
%     animStruct = drawAxes(animStruct,plotFrames,.1*ones(3,1));
%     drawnow;

    %%%%%%%%%%%%%%%%%%
    % Joystick Input %
    %%%%%%%%%%%%%%%%%%
    
    [axes, buttons, povs] = read(joy);
    
    joyLowPass = .98;
    axes = joyLowPass * axesLast + ...
                    (1-joyLowPass) * axes;
    axesLast = axes;   
    
    
    QUIT_BUTTON = 11;
    
    if buttons(QUIT_BUTTON)
        break;
    end
    
    % Chassis Fwd / Back Vel
    joyScale = -.6;
    joyDeadZone = .06;
    if abs(axes(6)) > joyDeadZone
        cmdVel = joyScale * (axes(6) - joyDeadZone*sign(axes(6)));
    else
        cmdVel = 0;
    end
    
    % Chassis Yaw Vel
    if abs(axes(3)) > joyDeadZone
        rotDiff = 25 * wheelRadius * direction(1) / wheelBase * ...
                        (axes(3) - joyDeadZone*sign(axes(3)));
    else
        rotDiff = 0;
    end
    
    % Comply to efforts from the arm
    armBaseTorque = mean(fbk.effort([armDOFs{1}(1),armDOFs{2}(1)]));
    rotCompDZ = 0.75;
    if abs(armBaseTorque) > rotCompDZ
        rotComp = 4 * (armBaseTorque - rotCompDZ*sign(armBaseTorque));
    else
        rotComp = 0;
    end
    rotDiff =  rotDiff + rotComp;
    
    % Stance Height
    if (buttons(10))
        %Use button 2 for a soft shutdown procedure
        kneeVelocity = 1.0;
        hipVelocity = kneeVelocity/2;
        %Lower robot until kneeAngle threshold before exiting
        if(kneeAngle > 2.5)
            break;
        end
    else
        % Normal Stance Height control
        if abs(axes(4)-axes(5)) > joyDeadZone  
            kneeVelocity = .5 * (axes(4)-axes(5));
            hipVelocity = kneeVelocity/2;  
        else
            kneeVelocity = 0;
            hipVelocity = 0;
        end
    end
    

        
    % Arm Y-Axis
    if abs(axes(1)) > joyDeadZone*3
        gripVel(2,:) = -.2 * axes(1);
    else
        gripVel(2,:) = 0;
    end
    
    % Arm X-Axis
    if abs(axes(2)) > joyDeadZone
        gripVel(1,:) = -.2 * axes(2);
    else
        gripVel(1,:) = 0;
    end
    
    % Arm Z-Axis
    if buttons(6)
        gripVel(3,:) = joyLowPass*gripVel(3,:) + (1-joyLowPass)*.2;
    elseif buttons(5)
        gripVel(3,:) = joyLowPass*gripVel(3,:) - (1-joyLowPass)*.2;
    else
        gripVel(3,:) = joyLowPass*gripVel(3,:);
    end
    
    % Wrist Rotation
    if povs == 0
        wristVel = joyLowPass*wristVel - (1-joyLowPass)*2.5;
    elseif povs == 180
        wristVel = joyLowPass*wristVel + (1-joyLowPass)*2.5;
    else
        wristVel = joyLowPass*wristVel;
    end
    
    % Chassis Pitch Rotation
    if povs == 90
        hipPitchVel = joyLowPass*hipPitchVel - (1-joyLowPass)*.1;
    elseif povs == 270
        hipPitchVel = joyLowPass*hipPitchVel + (1-joyLowPass)*.1;
    else
        hipPitchVel = joyLowPass*hipPitchVel;
    end

    
    %Camera Tilt
    %Triangle - 4, X - 2, O - 3, square - 1
    if buttons(3)
        camTiltPos = 1.4;
    elseif buttons(1)   
        camTiltPos = 0;
    else
        camTiltPos = nan;
        if buttons(4)
            camTiltVel = -0.6;
        elseif buttons(2)
            camTiltVel = 0.6;
        else
            camTiltVel = 0;
        end 
    end
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Control the Leg Positions %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Joint Limits
    kneeAngleMax = 2.65;
    kneeAngleMin = .65;
    
    if (kneeAngle > kneeAngleMax && kneeVelocity > 0) || ...
       (kneeAngle < kneeAngleMin && kneeVelocity < 0)
        kneeVelocity = 0;
        hipVelocity = 0;
    end
    
    hipPitch = hipPitch + hipPitchVel*dt;
    
    kneeAngle = kneeAngle + kneeVelocity * dt;
    hipAngle = pi/2 + kneeAngle/2 + hipPitch;
    

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Control the Arm End Effector Positions %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Make end effector velocities mirrored in Y, integrate velocity
    % commmands to get new positions.
    gripVel(2,2) = -gripVel(2,2);
    
%     % Used if keeping arms stationary in world frame
%     if armVelComp
%         gripVel(1,:) = gripVel(1,:) + fbkChassisVel;
%     end
    
    %gripVel = R_y(fbkLeanAngle) * gripVel;
    
    oldGripPos = gripPos;
    newGripPos = gripPos + gripVel*dt;
    gripWidthLim = .00;
    if newGripPos(2,1) < gripWidthLim
        newGripPos(2,:) = [gripWidthLim, -gripWidthLim];
    end
    
    % Use Jacobian inverse to get joint velocities from desired XYZ vel
    for i=1:numArms
        newArmJointAngs(:,i) = armKin{i}.getIK( ...
                                'xyz', newGripPos(:,i), ...
                                'initial', fbk.position(armDOFs{i}) );
        newArmJointVels(:,i) = J_armFbk{i}(1:3,:) \ gripVel(:,i);
                           
        J_new{i} = armKin{i}.getJacobianEndEffector( ...
                                newArmJointAngs(:,i) );                       
        det_J_new(i) = abs(det(J_new{i}(1:3,1:3)));
    end
    
    % Check manipulability to make sure arms don't go to singularity
    if (min(det_J_cmd) < J_limit) && min(det_J_new) < min(det_J_cmd)
        gripPos = oldGripPos;
        armJointAngs(1:3,:) = armJointAngs(1:3,:);
        armJointVels(1:3,:) = zeros(size(newArmJointVels(1:3,:)));
    else
        gripPos = newGripPos;
        armJointAngs(1:3,:) = newArmJointAngs(1:3,:);
        armJointVels(1:3,:) = newArmJointVels(1:3,:);
    end
    
    % Handle the wrist separately
    armJointVels(4,:) = armJointVels(2,:) + armJointVels(3,:) + ...
                            direction * wristVel;
    armJointAngs(4,:) = armJointAngs(4,:) + armJointVels(4,:)*dt;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Chassis Velocity Controller %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    cmdChassisVel = cmdVel;
    
    % PID Controller that set a desired lean angle
    velP = 5;
    velI = 3;
    velD = .3;
    
    fbkChassisVel = wheelRadius * mean(direction.*fbk.velocity(1:2)) + ...
                     heightCoM*leanAngleVel;
    
    chassisVelError = cmdChassisVel - fbkChassisVel;
    
    chassisVelErrorCum = chassisVelErrorCum + chassisVelError*dt;

    % chassisVelErrorCum = softStart * chassisVelErrorCum;
    chassisVelErrorCum = min(abs(chassisVelErrorCum),5/velI) * ...
                                    sign(chassisVelErrorCum);
                                
    cmdChassisAccel = (cmdChassisVel - cmdChassisVelLast) / dt;
    cmdChassisVelLast = cmdChassisVel;

    chassisAccel = (fbkChassisVel - fbkChassisVelLast) / dt;
    fbkChassisVelLast = fbkChassisVel;
    
    leanFF = 0.2 * robotMass * cmdChassisAccel / heightCoM;
    velFF = direction * cmdChassisVel / wheelRadius;
    
    cmdLeanAngle = velP * chassisVelError + ...
                   velI * chassisVelErrorCum + ...
                   velD * chassisAccel + ...
                   leanFF;


    % Reset the Command Struct                            
    cmd.effort = nan(1,numDOFs);
    cmd.position = nan(1,numDOFs);
    cmd.velocity = nan(1,numDOFs);
    
    
    % Lean Angle Control
    leanAngleError = fbkLeanAngle - cmdLeanAngle;
    leanAngleErrorLast = leanAngleError;

    leanAngleErrorCum = leanAngleErrorCum + leanAngleError * dt;
    leanAngleErrorCum = min(abs(leanAngleErrorCum),.2) * ...
                                    sign(leanAngleErrorCum);
                                        
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    % COMMANDS FOR THE WHEELS %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%               
    % Torques are commanded based lean angle controller
    % 
    % NOTE: Gains for the wheel modules are set so that only the FF term is
    % active in effort and velocity.  This lets you control PWM directly
    % while still working in more intuitive units for velocity and effort.
    if balanceOn         
        % PID Controller to servo to a desired lean angle
        leanP = 2;
        leanI = 20;
        leanD = 10;
    
        cmd.effort(wheelDOFs) = direction*leanP*leanAngleError + ...
                                direction*leanI*leanAngleErrorCum + ...
                                direction*leanD*leanAngleVel;  
                           
        cmd.effort(wheelDOFs) = softStart * cmd.effort(wheelDOFs);                
    end

    cmd.velocity(wheelDOFs) = rotDiff + velFF;
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % COMMANDS FOR THE LEGS %
    %%%%%%%%%%%%%%%%%%%%%%%%%
    for leg=1:numLegs
        cmd.position(legDOFs{leg}) = ...
                        direction(leg) * [ hipAngle kneeAngle];
        cmd.velocity(legDOFs{leg}) = ...
                        direction(leg) * [ hipVelocity kneeVelocity ];
    end
    
    % Impedence Control Params
    damperGains = [4; 0; 2; .0; .0; .0;]; % N or Nm / m/s
    springGains = [400; 0; 100; 0; 0; 0];  % N/m or Nm/rad
    
    rollGains = [0; 0; 10; 0; 0; 0];  % N or Nm / deg
    rollSign = [-1 1];
    
    for leg=1:numLegs
        % Impedence Control Torques
        legTipCmdFK = legKin{leg}.getFK( 'endeffector', ...
                                cmd.position(legDOFs{leg}) );
        xyzError = legTipCmdFK(1:3,4) - legTipFK{leg}(1:3,4);
        posError = [xyzError; zeros(3,1)];

        velError = J_legFbk{leg} * ( cmd.velocity(legDOFs{leg}) - ...   
                                fbk.velocity(legDOFs{leg}) )';                    

        impedanceTorque = J_legFbk{leg}' * ...
                        (springGains .* posError + ...
                         damperGains .* velError + ...
                         rollGains .* rollAngle*rollSign(leg)); 

        % Gravity Compensation Torques
        %gravCompTorque = J_legFbk{leg}(1:3,:)' * 9.8*[0; 0; -robotMass/4];
        gravCompTorque = zeros(2,1);
                            
        cmd.effort(legDOFs{leg}) = softStart*( impedanceTorque' + ...
                                                gravCompTorque' );                  
    end
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % COMMANDS FOR THE ARMS %
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Arms are controlled in a more traditional way with position /
    % velocity and efforts like with fixed base manipulation.
    
    % Impedence Control Params
    damperGains = [1; 1; 1; .0; .0; .0;]; % N or Nm / m/s
    springGains = [200; 20; 200; 0; 0; 0];  % N/m or Nm/rad
    
    for i = 1:numArms
        
        % Impedence Control Torques
        xyzError = newGripPos(:,i) - armTipFK{i}(1:3,4);
        posError = [xyzError; zeros(3,1)];
    
        velError = J_armFbk{i} * ( armJointVels(:,i)' - ...   
                                fbk.velocity(armDOFs{i}) )';                    
                            
        impedanceTorque = J_armFbk{i}' * ...
                        (springGains .* posError + ...
                         damperGains .* velError); 
                     
        % Gravity Compensation Torques
        gravCompTorque = armKin{i}.getGravCompEfforts( ...
                                fbk.position(armDOFs{i}), ...
                                -T_pose(3,1:3) );             
        
        % Fill in the appropriate part of the                     
        cmd.effort(armDOFs{i}) = softStart*impedanceTorque' + ...
                                                        gravCompTorque;
        cmd.position(armDOFs{i}) = armJointAngs(:,i);
        cmd.velocity(armDOFs{i}) = armJointVels(:,i);
    end
    
    cmd.velocity(15) = camTiltVel;
    cmd.position(15) = camTiltPos;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % SEND COMMANDS TO THE ROBOT %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    robotGroup.send(cmd);
    
    if logging
        timeHist(end+1,1) = fbk.time;
        cmdLeanAngleHist(end+1,1) = cmdLeanAngle;
        fbkLeanAngleHist(end+1,1) = fbkLeanAngle;
        leanAngleVelHist(end+1,1) = leanAngleVel;
        %leanAngleOffsetHist(end+1,1) = leanAngleOffset;
        cmdChassisVelHist(end+1,1) = cmdChassisVel;
        fbkChassisVelHist(end+1,1) = fbkChassisVel;
        RPYHist(end+1,:) = RPY;
    end
end

%%
%%%%%%%%%%%%
% PLOTTING %
%%%%%%%%%%%%
if logging
    log = struct(robotGroup.stopLogFull());
    timeHist = timeHist - timeHist(1);

    % PLOTTING
    figure(101);
    plot(timeHist,fbkLeanAngleHist,'b');
    hold on;
    plot(timeHist,cmdLeanAngleHist,'r');
    hold off;
    title('Lean Angle Tracking');
    xlabel('time (sec)');
    ylabel('angle (deg)');

    figure(102);
    plot(timeHist,fbkChassisVelHist,'b');
    hold on;
    plot(timeHist,leanAngleVelHist,'g');
    plot(timeHist,cmdChassisVelHist,'r');
    hold off;
    title('Chassis Velocity Tracking');
    xlabel('time (sec)');
    ylabel('velocity (m/sec)');

%     figure(103); 
%     fbkChassisAccel = smooth(diff(fbkChassisVelHist)./diff(timeHist),100);
%     plot(timeHist(2:end),5*fbkChassisAccel);
%     hold on;
%     fbkOffset = mean(fbkChassisAccel-fbkLeanAngleHist(2:end));
%     plot(timeHist,fbkLeanAngleHist+fbkOffset);
%     plot(timeHist,cmdLeanAngleHist+fbkOffset,'--');
%     hold off;
%     title('Lean Angle Debugging'
%     legend('Chassis Accel Debug','Feedback Lean Angle','Command Lean Angle');
%     xlabel('time (sec)');
%     ylabel('angle (deg)');

    % figure(104);
    % ax = subplot(1,1,1);
    % plot(log.time, direction .* log.velocity(:,wheelDOFs));
    % hold on;
    % ax.ColorOrderIndex = 1;
    % plot(log.time, direction .* log.velocityCmd(:,wheelDOFs),'--');
    % hold off;
end

