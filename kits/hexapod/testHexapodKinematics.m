% Prototype Hexapod Kinematics
%
% Dave Rollinson
% Feb 2017

clear *;
close all;
HebiLookup.clearModuleList();

simulate = true;

kin = makeHexapodKinematics();

for leg=1:length(kin)
    trajGen{leg} = HebiTrajectoryGenerator(kin{leg}); 
    trajGen{leg}.setAlgorithm('UnconstrainedQP');
end

moduleNames = { 'base1', 'shoulder1', 'elbow1', ...
                'base2', 'shoulder2', 'elbow2', ...
                'base3', 'shoulder3', 'elbow3', ...
                'base4', 'shoulder4', 'elbow4', ...
                'base5', 'shoulder5', 'elbow5', ...
                'base6', 'shoulder6', 'elbow6' };

if ~simulate
    group = HebiLookup.newGroupFromNames('xMonster',moduleNames);
    group.setFeedbackFrequency(200);
    pause(.2);
    fbk = group.getNextFeedback();

    poseFilter = HebiPoseFilter();
    poseFilter.setMaxAccelWeight( 0.05 );
    poseFilter.setMaxAccelNormDev( .5 );

    % Start the filter with 'clean' data to ensure it initializes well
    accel = [0.01; 0.01; 9.81];
    gyro = [0.01; 0.01; 0.01];
    poseFilter.update( accel, gyro, fbk.time );
end

setHexapodParams;

if ~simulate
    gainStruct = setHexapodGains( group, jointInds );
end

if simulate
    fbk.position = zeros(1,3*numLegs);
end

% Feedback for initial stance position
for leg=1:numLegs
    legEndPointFrame = kin{leg}.getFK( 'endeffector', ...
                           fbk.position(jointInds(leg,:)) ); 
    fbkStanceXYZ(:,leg) = legEndPointFrame(1:3,4); 
    seedAngles(:,leg) = fbk.position(jointInds(leg,:));
end
stanceXYZ = fbkStanceXYZ;

% PLOTTING FLAG
plotting = false;

animStruct.fig = figure(101);
animStruct2.fig = figure(102);
isFirstDraw = true;
axisLen = [.1 .1 .1];

% Approx home leg positions 
seedAngles = repmat( [.2 -.3 -1.9], numLegs, 1 );

timeHist = nan(0,1);
angHist = nan(0,3*numLegs);
angVelHist = nan(0,3*numLegs);
effortHist = nan(0,3*numLegs);
xyzVelHist = nan(0,3);
rotVelHist = nan(0,3);

frames = nan(4,4,6*6);
frameIndex = reshape(1:36,6,6);

legEfforts = nan(6,3);    
stanceLegs = 1:numLegs;
isStance = true(1,numLegs);
isFlight = ~isStance;
stepMode = true;
tocSwing = 0;

isCommandLeg = true(1,numLegs);

cmd = CommandStruct();
cmd.position = nan(1,3*numLegs);
cmd.velocity = nan(1,3*numLegs);
cmd.effort = nan(1,3*numLegs);

joy = HebiJoystick(1);
%joyMSI = HebiLookup.newGroupFromNames('*',{'MSI_IO_BOARD'});

%group.startLog();
mainTime = tic;
tocLast = toc(mainTime);
while true
    
    if ~simulate
        fbk = group.getNextFeedback();
    end
    
    % Timekeeping
    tocNow = toc(mainTime);
    dt = tocNow - tocLast;
    tocLast = tocNow;
    
    % Update Commanded Chassis Pose
    [xyzVel, rotVel, auxCmd] = getJoyCommands(joy);
    %[xyzVel, rotVel, auxCmd] = getMSIJoyCommands(joyMSI);
    
        
    if ~simulate
        % Update Feedback Chassis Orientation
        accelsModuleFrame = [ fbk.accelX(jointInds(:,1)); 
                              fbk.accelY(jointInds(:,1)); 
                              fbk.accelZ(jointInds(:,1)) ];
        gyrosModuleFrame = [ fbk.gyroX(jointInds(:,1)); 
                             fbk.gyroY(jointInds(:,1)); 
                             fbk.gyroZ(jointInds(:,1)) ];    

        for leg=[1 2]
            baseFrame = kin{leg}.getBaseFrame();
            accelsBodyFrame(:,leg) = baseFrame(1:3,1:3) * accelsModuleFrame(:,leg);
            gyrosBodyFrame(:,leg) = baseFrame(1:3,1:3) * gyrosModuleFrame(:,leg);
        end

        poseFilter.update( mean(accelsBodyFrame,2), mean(gyrosBodyFrame,2), fbk.time );
        T = poseFilter.getPose();
    else
        T = eye(4);
    end
    
%     animStruct2 = drawAxes( animStruct2, T, axisLen );
%     drawnow;
% 
%     if isFirstDraw
%         isFirstDraw = false;
%         axis equal;
%         grid on;
%         view(3);
%         plotLims = [-.11 .11];
%         xlim(plotLims);
%         ylim(plotLims);
%         zlim(plotLims);  
%     end

    
    if ~isempty(auxCmd) && auxCmd.quit
        disp('Quitting.');
        break;
    end
    
    if ~isempty(auxCmd) && auxCmd.toggleStepping
        stepMode = ~stepMode;
        pause(.1);  % Debounce
        if stepMode
            disp('Stepping Mode');
        else
            disp('Stance Mode');
        end
    end
    
    xyzVelHist(end+1,:) = xyzVel;
    rotVelHist(end+1,:) = rotVel;
    
    xyzTrans = xyzVel*dt;
    rotX = R_x( rotVel(1)*dt );
    rotY = R_y( rotVel(2)*dt );
    rotZ = R_z( rotVel(3)*dt );
    
    % Get linear velocities of stance legs based on rot/trans velocities
    stanceVelXYZ = xyzVel + cross(repmat(rotVel,1,numLegs),stanceXYZ);
    
    % Updated commanded stance positions 
    stanceXYZ = stanceXYZ + repmat(xyzTrans,1,numLegs);  % Translation
    stanceXYZ = rotZ * rotY * rotX * stanceXYZ; % Rotation
    
    % Feedback stance position
    for leg=1:numLegs
        legEndPointFrame = kin{leg}.getFK( 'endeffector', ...
                               fbk.position(jointInds(leg,:)) ); 
        fbkStanceXYZ(:,leg) = legEndPointFrame(1:3,4); 
    end
    
    % Make the home stance match the current Z-height
    levelHomeStanceXYZ(3,:) = levelHomeStanceXYZ(3,:) + xyzVel(3)*dt;
    homeStanceXYZ = R_x(.2*xyzVel(2)) * R_y(-.2*xyzVel(1)) * levelHomeStanceXYZ;
    
    % Calculate the difference between center-of-feet in 
    % current position and chassis center. 
    shiftPose = getBodyPoseFromFeet( fbkStanceXYZ, homeStanceXYZ );
    stanceShift = shiftPose(1:3,4);
    
    % Distance from feet to gravity vector
    gravityVec = -T(3,1:3)';
    gravityVecs = repmat(gravityVec,1,numLegs);
    stanceDots = repmat(gravityVec'*stanceXYZ,3,1);
    footDists = sqrt(sum((stanceDots.*gravityVecs - stanceXYZ).^2));

    footCoeffs = sum(footDists) ./ footDists;
    if any(isFlight)
        stanceSwitchTime = .1;
        if tocSwing < stanceSwitchTime
            switchBlendFactor = ...
                    (stanceSwitchTime - tocSwing) / stanceSwitchTime;
        elseif tocSwing > stepPeriod - stanceSwitchTime
             switchBlendFactor = ...
                    (tocSwing - (stepPeriod-stanceSwitchTime)) / ...
                    stanceSwitchTime;    
        else
            switchBlendFactor = 0;
        end
        footCoeffs(isFlight) = footCoeffs(isFlight) * switchBlendFactor;
    else
        switchBlendFactor = 1;
    end
    %footCoeffs = footCoeffs / (sum(footCoeffs) / (1 + switchBlendFactor));
    footCoeffs = footCoeffs / sum(footCoeffs);
    footCoeffs = footCoeffs * (1 + .33*sin(pi*switchBlendFactor));
       
    weightVec = robotWeight * gravityVecs;
    cmdFootForce = repmat(footCoeffs,3,1) .* weightVec;
    
    % Yaw
    RPY = SpinCalc( 'DCMtoEA123', ...
            shiftPose(1:3,1:3), 1E6, 0 ); 
    stanceYaw = RPY(3);
    if stanceYaw > 180
        stanceYaw = stanceYaw - 360;
    end
    stanceYaw = deg2rad(stanceYaw);
   
    % START A NEW STEP IF BODY IS OUT OF POSITION
    if ( norm(stanceShift(1:2))+norm(xyzVel(1:2))*stepPeriod > shiftThresh ...
         || abs(stanceYaw) > rotThresh ) && ~stepping && stepMode

        if legStepState == 0
            stepLegs = [1,4,5];
            stanceLegs = [2,3,6];
        else
            stepLegs = [2,3,6];
            stanceLegs = [1,4,5];
        end

        % update leg step state
        legStepState = rem(legStepState+1,2);
        
        % Step Phase and Stepping Foot Logical Masks
        tocLift = tocNow;
        
        isStance(allLegs) = true;
        isStance(stepLegs) = false;
        isFlight = ~isStance;
        
        stepping = true;
        
        stepComp = stepOverShoot * stepPeriod * stanceVelXYZ;
 
        % Step Waypoints, [LIFT UP] -> [HIGH POINT] -> [TOUCH DOWN]
        liftUpXYZ = stanceXYZ;
        
        % Try to touch down overshooting the stance error
        liftOffVelXYZ = stanceVelXYZ;
        
        swingTimeVec = stepPhase * stepPeriod;  
        isFirstStepTime = true;
           
    end
    
    % UPDATE SWING TIMER AND CHANGE STATE IF NECESSARY
    if stepping
        
        stepComp = stepOverShoot * stepPeriod * stanceVelXYZ;
        
        % Try to touch down overshooting the stance error
        touchDownXYZ = homeStanceXYZ - stepComp;
        midStepXYZ = .5*liftUpXYZ + .5*touchDownXYZ;
        midStepXYZ(3,:) = midStepXYZ(3,:) + stepHeight;
        
        tocSwingLast = tocSwing;
        tocSwing = tocNow - tocLift;
        
        if tocSwing > stepPeriod
            stepping = false;
            isStance(allLegs) = true;
            isFlight(allLegs) = false;
            stanceLegs = 1:numLegs;
            stanceXYZ(:,stepLegs) = touchDownXYZ(:,stepLegs);
        end
    end
    
    % CALCULATE LEG JOINT ANGLES
    for leg=1:numLegs
        
        %seedAngles(leg,:) = legAngles(leg,:);   % Last commands
        seedAngles(leg,:) = fbk.position(jointInds(leg,:)); % Latest feedback

        if isStance(leg)
            legAngles(leg,:) = kin{leg}.getIK( 'xyz', stanceXYZ(:,leg), ...
                                     'initial', seedAngles(leg,:) );
            dynamicCompEffort = zeros(1,3);                  
        else
            % REPLAN SWING EVERY TIME
            stepWaypoints{leg} = [ liftUpXYZ(:,leg)';
                                   midStepXYZ(:,leg)';
                                   touchDownXYZ(:,leg)' ];
                               
            for j=1:size(stepWaypoints{leg},1)             
                stepAngles{leg}(j,:) = kin{leg}.getIK( ...
                                     'xyz', stepWaypoints{leg}(j,:), ...
                                     'initial', seedAngles(leg,:) );
            end
            
            if tocSwing==0
                % Starting Step Vel/Accel based on liftOff Velocity
                stepAngVels{leg} = nan(size(stepAngles{leg}));
                J = kin{leg}.getJacobianEndEffector( stepAngles{leg}(1,:) );           
                stepAngVels{leg}(1,:) = J(1:3,:) \ liftOffVelXYZ(:,leg); 
                
                stepAngAccels{leg} = nan(size(stepAngles{leg}));
                stepAngAccels{leg}(1,:) = 0; 
            end
            
            % Ending Step Vel\Accel for Trajectory
            J = kin{leg}.getJacobianEndEffector( stepAngles{leg}(end,:) );
            stepAngVels{leg}(end,:) = J(1:3,:) \ stanceVelXYZ(:,leg);
            stepAngAccels{leg}(end,:) = 0;
            
            swingTraj{leg} = trajGen{leg}.newJointMove( stepAngles{leg}, ...
                                        'time', swingTimeVec, ...
                                        'velocities', stepAngVels{leg}, ...
                                        'accelerations', stepAngAccels{leg} );  
    
            [pos,vel,acc] = swingTraj{leg}.getState(tocSwing);
            legAngles(leg,:) = pos;
            
            dynamicCompEffort = kin{leg}.getDynamicCompEfforts( ...
                                    pos, pos, vel, acc ); 
        end              
          
        J = kin{leg}.getJacobian('endeffector',legAngles(leg,:)); 
        J_xyz = J(1:3,:);
        
        if isStance(leg)
            legAngVels(leg,:) = J_xyz \ stanceVelXYZ(:,leg);         
        else
            legAngVels(leg,:) = vel;
        end 
        
        frames(:,:,frameIndex(:,leg)) = ...
                    kin{leg}.getFK('output',legAngles(leg,:));

        gravCompEffort = kin{leg}.getGravCompEfforts( legAngles(leg,:), gravityVec );
        
        springShift = -2.5; %N-m
        dragShift = 1.5; % N-m / (rad/sec)
        springEffort = [0 springShift 0] + [0 dragShift 0] .* legAngVels(leg,:);
        stanceEffort = J_xyz' * cmdFootForce(:,leg);
        legEfforts(leg,:) = gravCompEffort + stanceEffort' + ...
                            dynamicCompEffort + springEffort;
          
        if isCommandLeg(leg)
            cmd.position(jointInds(leg,:)) = legAngles(leg,:);
            cmd.velocity(jointInds(leg,:)) = legAngVels(leg,:);
            cmd.effort(jointInds(leg,:)) = legEfforts(leg,:);
        end
    end
    
%     if exist('isFirstStepTime','var') && isFirstStepTime
%         isFirstStepTime = false;
%     end
    if ~simulate
        group.set(cmd);
    end
    
    % STATE HISTORY
    timeHist(end+1) = tocNow;
    angHist(end+1,:) = reshape(legAngles',1,[]);
    angVelHist(end+1,:) = reshape(legAngVels',1,[]);
    effortHist(end+1,:) = reshape(legEfforts',1,[]);

      
    % ANIMATION
    if plotting
        animStruct = drawAxes( animStruct, frames, axisLen );
        if isFirstDraw
            title('Hexapod Kinematics');
            view(3);
            xlabel('x (m)');
            ylabel('y (m)');
            zlabel('z (m)');
            axisLims = [-.6 .6];
            axis equal;
            xlim(axisLims);
            ylim(axisLims);
            zlim(axisLims);
            isFirstDraw = false;
        end
        drawnow;
    end        
end

log = group.stopLogFull();

%% PLOTTING COMMAND HISTORY - SPACE BAR TO ADVANCE THRU LEGS
figure(42);

for plotLeg=1:numLegs
    % Position
    ax = subplot(3,1,1);
    plot(timeHist,angHist(:,jointInds(plotLeg,:)),'--');
    set(ax,'colorOrderIndex',1);
    if isCommandLeg(plotLeg)
        hold on;
        plot(log.time,log.position(:,jointInds(plotLeg,:)));
        hold off;
    end
    title(['Joint Angles - Leg: ' num2str(plotLeg)]);
    xlabel('time (sec)');
    ylabel('angles (rad)');
    xlim([timeHist(1) timeHist(end)]);
    ylim([-3.5 3.5]);
    grid on;
    
    % Velocity
    ax = subplot(3,1,2);
    plot(timeHist,angVelHist(:,jointInds(plotLeg,:)),'--');
    set(ax,'colorOrderIndex',1);
    if isCommandLeg(plotLeg)
        hold on;
        plot(log.time,log.velocity(:,jointInds(plotLeg,:)));
        hold off;
    end
    title(['Joint Velocities - Leg: ' num2str(plotLeg)]);
    xlabel('time (sec)');
    ylabel('velocity (rad/sec)');
    xlim([timeHist(1) timeHist(end)]);
    ylim([-4 4]);
    grid on;
    
    % Effort
    ax = subplot(3,1,3);
    plot(timeHist,effortHist(:,jointInds(plotLeg,:)),'--');
    set(ax,'colorOrderIndex',1);
    if isCommandLeg(plotLeg)
        hold on;
        plot(log.time,log.effort(:,jointInds(plotLeg,:)));
        hold off;
    end
    title(['Joint Efforts - Leg: ' num2str(plotLeg)]);
    xlabel('time (sec)');
    ylabel('effort (N-m)');
    xlim([timeHist(1) timeHist(end)]); 
    ylim([-10 15]);
    grid on;
  
    pause;
end

%% SIMULATE MOTOR POWER DISSIPATION

gearRatios = [762.22 1742.22 762.22];
gearRatios = repmat(gearRatios,1,numLegs);
motorTorqueConst = .00626 * gearRatios * .75;  % (Nm/A) Maxon * Ratio * Eff
motorResistance = 12; % (Ohm) Maxon

jointCurrent = effortHist ./ motorTorqueConst;
jointPowerTheory = jointCurrent.^2 * motorResistance;
jointPowerTheory(abs(jointPowerTheory)>500) = nan;
jointPowerActual = log.motorCurrent .* log.voltage;
jointPowerActual(abs(jointPowerActual)>500) = nan;

figure(43);
subplot(2,1,1);
plot(timeHist,jointPowerTheory);
title('Joint Power Draw');
xlabel('time (sec)');
ylabel('power (W)');
ylim([0 300]);


subplot(2,1,2);
hold off;
plot(timeHist,sum(jointPowerTheory,2));
hold on;
plot(log.time,smooth(sum(jointPowerActual,2)));
xlabel('time (sec)');
ylabel('power (W)');
ylim([0 300]);
legend('predicted','actual');


% %% COMPARE COMMANDED / ACTUAL VELOCITIES
% figure(456);
% plotModule = 6;
% plot(log.time(2:end),smooth(diff(log.positionCmd(:,plotModule)))./smooth(diff(log.time)))
% hold on;
% plot(log.time,log.velocityCmd(:,plotModule))
% hold off;