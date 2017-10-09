% Testing out balancing robot control.
%
% Assumes using a Sony PS4 Gamepad:
% Model CUH-ZCT2U Wireless Controller
%
% Dave Rollinson
% Apr 2017

clear *;
close all;

moduleNames = { '_port', '_starboard', ...
                'base1', 'shoulder1', 'elbow1', ...
                'base2', 'shoulder2', 'elbow2' };
robotGroup = HebiLookup.newGroupFromNames('Igor',moduleNames);
robotGroup.setFeedbackFrequency(500);
pause(2.0);
fbk = robotGroup.getNextFeedbackFull();
timeLast = fbk.time;

balanceOn = true;
logging = true;   % Flag to turn logging on and off.  If logging is on you 
                  % you can view a bunch of debug plots after quitting.
 
numArms = 2;
wheelDOFs = [1 2];
armDOFs{1} = [3 4 5];
armDOFs{2} = [6 7 8];
imuModules = [ wheelDOFs armDOFs{1}(1) armDOFs{2}(1) ];

% WHEELS OUT
R_port = R_z(pi)*R_x(pi/2);
R_starboard = R_x(pi/2);
direction = [-1 1];

% % WHEELS IN
% R_starboard = R_z(pi)*R_x(pi/2);
% R_port = R_x(pi/2);
% direction = [1 -1];

wheelRadius = .200 / 2;  % m
wheelBase = .600;  % m

J_limit = .020;

chassisCoM = [0; 0; .25];  % m
chassisMass = 7.0;  % kg

armVelComp = false;

% Setup kinematics and create trajectory generator
chassisKin = HebiKinematics();
chassisKin.addBody('X5-4');

% Setup Arms
R_hip = R_x(pi/2);
xyz_hip = [0; .0225; .055]; 
T_hip = eye(4);
T_hip(1:3,1:3) = R_hip;
T_hip(1:3,4) = xyz_hip;

armBaseXYZ(:,1) = [0; .12; .43];
armBaseXYZ(:,2) = [0; -.12; .43];

for arm = 1:numArms
    armKin{arm} = HebiKinematics();
    armKin{arm}.addBody('X5-4');
    armKin{arm}.addBody( 'GenericLink', 'com', xyz_hip/2, 'out', T_hip, 'mass', .225 );
    armKin{arm}.addBody('X5-9');
    armKin{arm}.addBody('X5Link','ext',.29,'twist',pi,'mass',.250);
    armKin{arm}.addBody('X5-4');
    armKin{arm}.addBody('X5Link','ext',.39,'twist',0,'mass',.350);
    armTransform = eye(4);
    armTransform(1:3,4) = armBaseXYZ(:,arm);
    armKin{arm}.setBaseFrame(armTransform);
end

% armHomeAngles(:,1) = deg2rad([180 180+35 -45])';
% armHomeAngles(:,2) = deg2rad([0 -35 45])';

armHomeAngles(:,1) = fbk.position(armDOFs{1});
armHomeAngles(:,2) = fbk.position(armDOFs{2});

for arm = 1:numArms
    T_endEffector = armKin{arm}.getFK('endeffector',armHomeAngles(:,arm));
    gripPos(:,arm) = T_endEffector(1:3,4);
end

gripPos(2,2) = -gripPos(2,2);
gripPos = repmat(mean(gripPos,2),1,2);
gripPos(2,2) = -gripPos(2,2);

fbkChassisVel = 0;
velNow = 0;
accNow = 0;
jerkNow = 0;

maxAccel = .25;

payloadLowPass = .99;
baselineForce = zeros(3,numArms);
payloadMass = 0;

joy = HebiJoystick(1);
[axes, buttons, povs] = read(joy);
axesLast = axes;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Keep initializing Pose Filter until it works %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T_init = nan(4);

while any(isnan(T_init(:)))
    disp('Initializing filter.');
    fbk = robotGroup.getNextFeedbackFull();
    filterTime = fbk.time;
    
    accel = [fbk.accelX; fbk.accelY; fbk.accelZ];
    gyro = [fbk.gyroX; fbk.gyroY; fbk.gyroZ];
    
    poseFilter = HebiPoseFilter();
    poseFilter.setGyroScale( 1E-6 );
    poseFilter.setMaxAccelWeight( .005 );
    poseFilter.setMaxAccelNormDev( .3 );
    
    poseAccel = accel(:,imuModules);
    poseGyro = gyro(:,imuModules);
    poseAccel(:,1) = R_port * poseAccel(:,1);
    poseGyro(:,1) = R_port * poseGyro(:,1);
    poseAccel(:,2) = R_starboard * poseAccel(:,2);
    poseGyro(:,2) = R_starboard * poseGyro(:,2);
    
    poseAccel = mean(poseAccel,2);
    poseGyro = mean(poseGyro,2);
    
    poseFilter.update( poseAccel, poseGyro, filterTime );

    T_init = poseFilter.getPose();
    
end

T_pose = T_init;

animStruct.fig = figure(101);
axisLen = .1*ones(3,1);
isFirstDraw = true;

cmd = CommandStruct();
armCmd = CommandStruct();

gripVel = zeros(3,numArms);

leanAngleErrorVelLast = 0;
fbkChassisVelLast = 0;
cmdChassisVelLast = 0;

if logging
    robotGroup.startLog;
end

timeHist = nan(0,1);
cmdLeanAngleHist = nan(0,1);
fbkLeanAngleHist = nan(0,1);
leanAngleOffsetHist = nan(0,1);
cmdChassisVelHist = nan(0,1);
fbkChassisVelHist = nan(0,1);

leanAngleErrorCum = 0;
leanAngleErrorLast = 0;
chassisVelErrorCum = 0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Intial Trajectory for Wheels %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cmdVel = 0;
minRampTime = .33;
rampTime = max(abs(cmdVel-velNow)/maxAccel,minRampTime);


time = [ 0 rampTime ];
vels = [velNow cmdVel];
accels = [accNow 0 ];
jerks = [jerkNow 0];

trajGen = HebiTrajectoryGenerator(chassisKin);
trajGen.setSpeedFactor(1);
trajGen.setAlgorithm('UnconstrainedQp');

trajectory = trajGen.newJointMove( vels, ...
            'Velocities', accels, ...
            'Accelerations', jerks, ...
            'Time', time );
trajTimer = tic;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CONTROL LOOP                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
while true
    
    %%%%%%%%%%%%%%%%
    % Time Keeping %
    %%%%%%%%%%%%%%%%
    fbk = robotGroup.getNextFeedback(fbk);
    dt = mean(fbk.time - timeLast);
    dt = min(dt,.02);
    timeLast = fbk.time;
    filterTime = filterTime + dt;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Compensate Balance based on arm config and payload estimate %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Arm Kinematics
    for i=1:numArms
        armFK = armKin{i}.getFK('com',fbk.position(armDOFs{i}));
        armTipFK{i} = armKin{i}.getFK( 'endeffector', ...
                                fbk.position(armDOFs{i}) );
        J_fbk{i} = armKin{i}.getJacobianEndEffector( ...
                                fbk.position(armDOFs{i}) );                       
        det_J_fbk(i) = abs(det(J_fbk{i}(1:3,:)));        
        
        J_cmd{i} = armKin{i}.getJacobianEndEffector( ...
                                fbk.positionCmd(armDOFs{i}) );
        det_J_cmd(i) = abs(det(J_cmd{i}(1:3,:))); 
        
        armXYZ = squeeze(armFK(1:3,4,:));
        armMasses = armKin{i}.getBodyMasses;
        armCoM(:,i) = sum( armXYZ.*armMasses', 2 ) / sum(armMasses);
        armMass(i) = sum(armMasses);
    end
    
    % Get the estimated payload force from Jacobian transpose inverse
    if ~isnan(fbk.torqueCmd)
        for i=1:numArms
            payloadForce(:,i) = J_fbk{i}(1:3,:)' \ ...
                    (fbk.torque(armDOFs{i}) - fbk.torqueCmd(armDOFs{i}))';
            payloadCoM = armTipFK{i}(1:3,4);
        end
    else
        payloadForce = zeros(3,numArms);
        payloadCoM = zeros(3,1);
        payloadMass = 0;
    end
    
    % Do a gradual update of the force and only take the component that is
    % aligned with gravity
    if mean(abs(payloadForce(2,:))) > 2 && (min(det_J_fbk) > J_limit)
       netForce = numArms * mean(payloadForce-baselineForce,2);
       forceVec = netForce / norm(netForce);
       
       gravityVec = T_pose(3,1:3);
       gravityScale = abs(dot(forceVec,gravityVec));
       
       newMass = gravityScale * norm(netForce) / 9.81;
       
       payloadMass = payloadLowPass * payloadMass + ...
                        (1-payloadLowPass) * newMass;
    elseif mean(abs(payloadForce(2,:))) < 2
       baselineForce = payloadLowPass * baselineForce + ...
                        (1-payloadLowPass) * payloadForce;
       payloadMass = payloadLowPass * payloadMass + ...
                        (1-payloadLowPass) * 0;
    end
   
    % Adjust CoM based on chassis, arm configuration, estimated payload.
    allCoMs = [armCoM chassisCoM payloadCoM];
    allMasses = [armMass chassisMass payloadMass];
    
    robotCoM = sum(allCoMs .* allMasses, 2) / sum(allMasses);
    robotMass = sum(allMasses);
    
    heightCoM = norm(robotCoM);
    
    leanAngleOffset = rad2deg(atan2(robotCoM(1),robotCoM(3)));

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Update date filtered pose estimate %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    accel = [fbk.accelX; fbk.accelY; fbk.accelZ];
    gyro = [fbk.gyroX; fbk.gyroY; fbk.gyroZ];
    
    poseAccel = accel(:,imuModules);
    poseGyro = gyro(:,imuModules);
    poseAccel(:,1) = R_port * poseAccel(:,1);
    poseGyro(:,1) = R_port * poseGyro(:,1);
    poseAccel(:,2) = R_starboard * poseAccel(:,2);
    poseGyro(:,2) = R_starboard * poseGyro(:,2);
    
    poseAccel = mean(poseAccel,2);
    poseGyro = mean(poseGyro,2);
    
    poseFilter.update( poseAccel, poseGyro, filterTime );

    T_pose = poseFilter.getPose();

    RPY = SpinCalc('DCMtoEA123',T_pose(1:3,1:3),1E-9,0);
    rollAngle = RPY(2);
    if rollAngle > 180
        rollAngle = rollAngle - 360;
    end
    
    fbkLeanAngle = rollAngle - leanAngleOffset;
    
    [axes, buttons, povs] = read(joy);
    
    joyLowPass = .98;
    axes = joyLowPass * axesLast + ...
                    (1-joyLowPass) * axes;
    axesLast = axes;   
    
    
    %%%%%%%%%%%%%%%%%%
    % Joystick Input %
    %%%%%%%%%%%%%%%%%%
    
    % End effector velocity compensation
    BALANCE_ON = 3;
    BALANCE_OFF = 12;
    VELCOMP_ON = 2;
    VELCOMP_OFF = 1;
    QUIT_BUTTON = 11;
    
    if buttons(VELCOMP_ON)
        armVelComp = true;
    end
    if buttons(VELCOMP_OFF)
        armVelComp = false;
    end
    
    if buttons(BALANCE_ON)
        balanceOn = true;
    end
    if buttons(BALANCE_OFF)
        balanceOn = false;
    end
    
    if buttons(QUIT_BUTTON)
        break;
    end
    
    % Chassis Fwd / Back Vel
    joyScale = .4;
    joyDeadZone = .06;
    if abs(axes(6)) > joyDeadZone
        cmdVel = joyScale * (axes(6) - joyDeadZone*sign(axes(6)));
    else
        cmdVel = 0;
    end
    
    % Chassis Yaw Vel
    if abs(axes(3)) > joyDeadZone
        rotDiff = 10 * wheelBase * direction(2) * ...
                        (axes(3) - joyDeadZone*sign(axes(3)));
    else
        rotDiff = 0;
    end
    
    armBaseTorque = mean(fbk.torque([armDOFs{1}(1),armDOFs{2}(1)]));
    rotCompDZ = .75;
    if abs(armBaseTorque) > rotCompDZ
        rotComp = 2 * (armBaseTorque - rotCompDZ*sign(armBaseTorque));
    else
        rotComp = 0;
    end
    rotDiff =  rotDiff + rotComp;
    
    % Arm Z-Axis
    if abs(axes(4)-axes(5)) > joyDeadZone
        gripVel(3,:) = -.1 * (axes(4)-axes(5));
    else
        gripVel(3,:) = 0;
    end
    
    % Arm Y-Axis
    if abs(axes(1)) > joyDeadZone
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
    
    % Arm Z-Axis Diff
    gripDiffZ = [.05 -.05];
    if buttons(6)
        gripVel(3,:) = gripVel(3,:) + gripDiffZ;
    elseif buttons(5)
        gripVel(3,:) = gripVel(3,:) - gripDiffZ;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Control the Arm End Effector Positions %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Make end effector velocities mirrored in Y, integrate velocity
    % commmands to get new positions.
    gripVel(2,2) = -gripVel(2,2);
    
    % Used if keeping arms stationary in world frame
    if armVelComp
        gripVel(1,:) = gripVel(1,:) + fbkChassisVel;
    end
    
    velCompAngle = deg2rad(rollAngle+leanAngleOffset);
    gripVel = R_y(velCompAngle) * gripVel;
    
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
        newArmJointVels(:,i) = J_fbk{i}(1:3,:) \ gripVel(:,i);
                           
        J_new{i} = armKin{i}.getJacobianEndEffector( ...
                                newArmJointAngs(:,i) );                       
        det_J_new(i) = abs(det(J_new{i}(1:3,:)));
    end
    
    % Check manipulability to make sure arms don't go to singularity
    if (min(det_J_cmd) < J_limit/2) && (min(det_J_new) < min(det_J_cmd))
        gripPos = oldGripPos;
        armJointAngs = armJointAngs;
        armJointVels = zeros(size(newArmJointVels));
    else
        gripPos = newGripPos;
        armJointAngs = newArmJointAngs;
        armJointVels = newArmJointVels;
    end
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%
    % Chassis Trajectories %
    %%%%%%%%%%%%%%%%%%%%%%%%
    
    % Smooth-ish trajectories for chassis velocity, replan evey timestep
    t = toc(trajTimer);
    [velNow, accNow, jerkNow] = trajectory.getState(t);

    rampTime = max(abs(cmdVel-velNow)/maxAccel,minRampTime);

    time = [ 0 rampTime ];
    vels = [velNow cmdVel];
    accels = [accNow 0 ];
    jerks = [jerkNow 0];

    trajGen = HebiTrajectoryGenerator(chassisKin);
    trajGen.setSpeedFactor(1);
    trajGen.setAlgorithm('UnconstrainedQp');

    trajectory = trajGen.newJointMove( vels, ...
                'Velocities', accels, ...
                'Accelerations', jerks, ...
                'Time', time );
    trajTimer = tic;
    
    cmdChassisVel = velNow;
    cmdChassisAccel = accNow;
    cmdChassisVelLast = cmdChassisVel;
   
 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Chassis Velocity Controller %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % PID Controller that set a desired lean angle
    velP = 4;
    velI = 3;
    velD = .05;
    
    leanAngleVel = mean(direction .* fbk.gyroZ(1:2));
    yawVel = mean(fbk.gyroY(1:2));
    
    fbkChassisVel = wheelRadius * mean(direction.*fbk.velocity(1:2)) + ...
                     heightCoM*leanAngleVel;
    
    chassisVelError = cmdChassisVel - fbkChassisVel;
    
    chassisVelErrorCum = chassisVelErrorCum + chassisVelError*dt;
    chassisVelErrorCum = min(abs(chassisVelErrorCum),4/velI) * ...
                                    sign(chassisVelErrorCum);
                                
    chassisAccel = (fbkChassisVel - fbkChassisVelLast) / dt;
    fbkChassisVelLast = fbkChassisVel;
    
    leanFF = 3 * cmdChassisAccel;
    velFF = 1 * direction * cmdChassisVel / wheelRadius;
    
    cmdLeanAngle = velP * chassisVelError + ...
                   velI * chassisVelErrorCum + ...
                   velD * chassisAccel + ...
                   leanFF;

    % View integral windup term, uncomment for debugging, 
    %velI * chassisVelErrorCum         
               
    leanAngleError = fbkLeanAngle - cmdLeanAngle;
    leanAngleErrorLast = leanAngleError;
    leanAngleErrorCum = leanAngleErrorCum + leanAngleError * dt;
    leanAngleErrorCum = min(abs(leanAngleErrorCum),.5) * ...
                                    sign(leanAngleErrorCum);

    % Rest the Command Struct                            
    cmd.torque = nan(1,8);
    cmd.position = nan(1,8);
    cmd.velocity = nan(1,8);
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    % COMMANDS FOR THE WHEELS %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%               
    % Torques are commanded based lean angle controller
    % 
    % NOTE: Gains for the wheel modules are set so that only the FF term is
    % active in torque and velocity.  This lets you control PWM directly
    % while still working in more intuitive units for velocity and torque.
    if balanceOn         
        % PID Controller to servo to a desired lean angle
        leanP = 2;
        leanI = 20;
        leanD = 3;
    
        cmd.torque(wheelDOFs) = direction*leanP*leanAngleError + ...
                     direction*leanI*leanAngleErrorCum + ...
                     direction*leanD*leanAngleVel;
    end
        
    % Velocities to wheels are feedforward based on desired chassis
    % velocity and desired yaw rotation velocity
    cmd.velocity(wheelDOFs) = rotDiff + velFF;

    
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % COMMANDS FOR THE ARMS %
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Arms are controlled in a more traditional way with position /
    % velocity and torques like with fixed base manipulation.
    
    % Impedence Control Params
    damperGains = [1; 1; 1; .0; .0; .0;]; % N or Nm / m/s
    springGains = [100; 10; 100; 0; 0; 0];  % N/m or Nm/rad
    
    for i = 1:numArms
        
        % Impedence Control Torques
        xyzError = newGripPos(:,i) - armTipFK{i}(1:3,4);
        posError = [xyzError; zeros(3,1)];
    
        velError = J_fbk{i} * ( armJointVels(:,i)' - ...   
                                fbk.velocity(armDOFs{i}) )';                    
                            
        impedanceTorque = J_fbk{i}' * ...
                        (springGains .* posError + ...
                         damperGains .* velError); 
                     
        % Gravity Compensation Torques
        gravCompTorque = armKin{i}.getGravCompTorques( ...
                                fbk.position(armDOFs{i}), ...
                                T_pose(3,1:3) );             
        
        % Fill in the appropriate part of the                     
        cmd.torque(armDOFs{i}) = impedanceTorque' + gravCompTorque;
        cmd.position(armDOFs{i}) = armJointAngs(:,i);
        cmd.velocity(armDOFs{i}) = armJointVels(:,i);
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % SEND COMMANDS TO THE ROBOT %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    robotGroup.set(cmd);
    
    if logging
        timeHist(end+1,1) = fbk.time;
        cmdLeanAngleHist(end+1,1) = cmdLeanAngle;
        fbkLeanAngleHist(end+1,1) = fbkLeanAngle;
        leanAngleOffsetHist(end+1,1) = leanAngleOffset;
        cmdChassisVelHist(end+1,1) = cmdChassisVel;
        fbkChassisVelHist(end+1,1) = fbkChassisVel;
    end
end


%%
%%%%%%%%%%%%
% PLOTTING %
%%%%%%%%%%%%
if logging
    log = struct(robotGroup.stopLogFull());
    timeHist = timeHist - timeHist(1);

    %% PLOTTING
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
    plot(timeHist,cmdChassisVelHist,'r');
    hold off;
    title('Chassis Velocity Tracking');
    xlabel('time (sec)');
    ylabel('velocity (m/sec)');

    figure(103);
    fbkChassisAccel = smooth(diff(fbkChassisVelHist)./diff(timeHist),100);
    plot(timeHist(2:end),5*fbkChassisAccel);
    hold on;
    fbkOffset = mean(fbkChassisAccel-fbkLeanAngleHist(2:end));
    plot(timeHist,fbkLeanAngleHist+fbkOffset);
    plot(timeHist,cmdLeanAngleHist+fbkOffset,'--');
    hold off;

    % figure(104);
    % ax = subplot(1,1,1);
    % plot(log.time, direction .* log.velocity(:,wheelDOFs));
    % hold on;
    % ax.ColorOrderIndex = 1;
    % plot(log.time, direction .* log.velocityCmd(:,wheelDOFs),'--');
    % hold off;
end

