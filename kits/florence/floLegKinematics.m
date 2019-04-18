% Kinematics for "Florence" Biped Kit
%
% Dave Rollinson
% Feb 2019

clear *;

kb = HebiKeyboard();

visualizeOn = true;
simulate = true;
logging = false;

robotName = 'Florence';
controllerName = '_Controller';
  
if visualizeOn
    xyzFrameLimits = [ -0.5  0.3;
                       -0.4  0.4;
                       -0.8  0.2 ];
    framesDisplay = FrameDisplay( [], [], xyzFrameLimits );
end

[ groups, kin, params ] = setupFlorence( robotName, controllerName, simulate );

legIndex = params.legIndex;

if ~simulate
    legsGroup = groups.legs;
    feetGroup = groups.feet;
    legGains = params.legGains;
    cmdGains = params.cmdGains;
    cmd = CommandStruct();
end

stanceHeight = -params.stanceHeight;    % [m]
stanceY = params.stanceWidth/2;    % [m]
stanceSkew = params.stanceSkew;    % [m]
stanceMidX = params.stanceMidX;    % [m]

stepPeriod = params.stepPeriod;  % [sec]
strideLength = params.strideLength;  % [m]
swingLiftHeight = params.swingLiftHeight;  % [m]

stanceStartX = stanceMidX + strideLength/2;  % [m]
stanceEndX = stanceMidX - strideLength/2;    % [m]

stanceXYZ(:,:,1) = [ stanceStartX  stanceY             stanceHeight;
                     stanceMidX    stanceY-stanceSkew  stanceHeight;
                     stanceEndX    stanceY             stanceHeight ];

stanceXYZ(:,:,2) = [ stanceStartX -(stanceY)            stanceHeight;
                     stanceMidX   -(stanceY-stanceSkew) stanceHeight;
                     stanceEndX   -(stanceY)            stanceHeight ];                 

swingXYZ = flipud(stanceXYZ);
swingXYZ(2,2,1) = swingXYZ(2,2,1)+stanceSkew;
swingXYZ(2,2,2) = swingXYZ(2,2,2)-stanceSkew;
swingXYZ(2,3,:) = swingXYZ(2,3,:) + swingLiftHeight;

stanceXYZ(2,3,:) = stanceXYZ(2,3,:) - swingLiftHeight;

stanceTiming = [0; 0.5*stepPeriod; stepPeriod];
swingTiming = [0; params.swingMidpointPhase*stepPeriod; stepPeriod];

stanceVelXYZ = nan(size(stanceXYZ));
%stanceVelXYZ = (stanceXYZ(3,:,:) - stanceXYZ(1,:,:)) / stepPeriod;

stanceVelXYZ([1,3],:,:) = diff(stanceXYZ) ./ diff(stanceTiming);

homeAngles = params.homeAngles;

footRot = params.footRot;

gravityVec = params.gravityVec;
chassisFrame = params.chassisFrame;
chassisMass = params.chassisMass;
chassisCoM = params.chassisCoM;

legSign = [-1 1];

% Get IK to XYZ waypoints
for leg=1:2

    for i=1:3
        stanceAngles(i,:,leg) = kin.leg{leg}.getIK( ...
                                        'xyz', stanceXYZ(i,:,leg), ...
                                        'SO3', footRot(:,:,leg), ...
                                        'initial', homeAngles(leg,:) );
        swingAngles(i,:,leg) = kin.leg{leg}.getIK( ...   
                                        'xyz', swingXYZ(i,:,leg), ...
                                        'SO3', footRot(:,:,leg), ...
                                        'initial', homeAngles(leg,:) );
        J_stance = kin.leg{leg}.getJacobian( 'endeffector', ...
                                         stanceAngles(i,:,leg) );
        stanceAngVels(i,:,leg) = J_stance \ [stanceVelXYZ(i,:,leg)'; 0; 0; 0];                             
    end
    
    swingAngVels(:,:,leg) = flipud(stanceAngVels(:,:,leg));
    swingAngVels(2,:,leg) = nan;
    stanceAngVels(2,:,leg) = nan;
    
end

stanceAngAccels = nan(size(stanceAngVels));
stanceAngAccels([1,3],:,:) = 0;
swingAngAccels = nan(size(swingAngVels));
swingAngAccels([1,3],:,:) = 0;

for leg=1:2
    trajGen{leg} = HebiTrajectoryGenerator(kin.leg{leg});
    
    stanceTraj{leg} = trajGen{leg}.newJointMove( ...
                                    stanceAngles(:,:,leg), ...
                                    'velocities', stanceAngVels(:,:,leg), ...
                                    'accelerations', stanceAngAccels(:,:,leg), ...
                                    'Time', stanceTiming );
    swingTraj{leg} = trajGen{leg}.newJointMove( ...
                                    swingAngles(:,:,leg), ...
                                    'velocities', swingAngVels(:,:,leg), ...
                                    'accelerations', swingAngAccels(:,:,leg), ...
                                    'Time', swingTiming );                              
end

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Calculate stance/swing trajectories for one leg %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
time = 0:0.001:stanceTraj{1}.getDuration();
[posStance,velStance,accStance] = stanceTraj{1}.getState(time);
[posSwing,velSwing,accSwing] = swingTraj{1}.getState(time);

xyzStance = nan(3,length(time));
xyzSwing = nan(3,length(time));


for i=1:length(time)
    
    % Do FK / Jacobians
    for leg = 1:2
        if leg==1
            [pos,vel,acc] = stanceTraj{leg}.getState(time(i));
        else
            [pos,vel,acc] = swingTraj{leg}.getState(time(i));
        end

        legFrames{leg} = kin.leg{leg}.getFK( 'output', pos );
        footFrame{leg} = legFrames{leg}(:,:,end);
        legCoMs{leg} = kin.leg{leg}.getFK( 'com', pos );
        
        % gravCompEfforts{leg} = kin.leg{leg}.getGravCompEfforts( pos, gravityVec);
        % dynamicCompEfforts{leg} = kin.leg{leg}.getDynamicCompEfforts( pos, pos, vel, acc );

        J{leg} = kin.leg{leg}.getJacobianEndEffector( pos );
        
        legXYZ = squeeze(legCoMs{leg}(1:3,4,:));
        legMasses = kin.leg{leg}.getBodyMasses;
        legCoM(:,leg) = sum( legXYZ.*repmat(legMasses',3,1), 2 ) / sum(legMasses);
        legMass(leg) = sum(legMasses);
                  
        legCmdAngles(leg,:) = pos;
        legCmdAngVels(leg,:) = vel;

    end
    
    % Find CoM based on chassis, leg configuration
    allCoMs = [legCoM chassisCoM];
    allMasses = [legMass chassisMass];

    robotCoM = sum(allCoMs .* repmat(allMasses,3,1), 2) / sum(allMasses);
    robotMass = sum(allMasses);  
    
    for leg = 1
        % Stance Leg
%         gravCompEfforts = kin.leg{leg}.getGravCompEfforts( posStance(i,:), gravityVec);
%         dynamicCompEfforts = kin.leg{leg}.getDynamicCompEfforts( ...
%                                                 posStance(i,:), ...
%                                                 posStance(i,:), ...
%                                                 velStance(i,:), ...
%                                                 accStance(i,:) );
% 
%         J_footFixedBody = kin.leg{leg}.getJacobianEndEffector( posStance(i,:) );
        
        % Jacobian to the pelvis in the foot-fixed frame
        J_footFixedFoot = getJacobianFixedEndEffectorFast( ...
                                   kin.leg{leg}, legCmdAngles(legIndex{leg} ) );

        legFK = kin.leg{leg}.getFK( 'endeffector', posStance(i,:) );
        
        xyzStance(:,i) = legFK(1:3,4);

        legMass = sum(kin.leg{leg}.getBodyMasses);
        robotMass = 2*legMass + chassisMass;

        stanceWrench = zeros(6,1);
        supportForce = 0.75 * (-9.8 * robotMass * gravityVec);
        
        stanceWrench(1:3) = supportForce;     
        stanceWrench(4:6) = cross(-supportForce,robotCoM);
        
        stanceWrench(1:3) = footFrame{leg}(1:3,1:3) * stanceWrench(1:3);
        stanceWrench(4:6) = footFrame{leg}(1:3,1:3) * stanceWrench(4:6);
        
        stanceEfforts = J_footFixedFoot' * stanceWrench;
        
        %effStance(i,:) = stanceEfforts + gravCompEfforts' + dynamicCompEfforts';
        effStance(i,:) = stanceEfforts;

        % Swing Leg (no stance torques)
        gravCompEfforts = kin.leg{leg}.getGravCompEfforts( posSwing(i,:), gravityVec);
        dynamicCompEfforts = kin.leg{leg}.getDynamicCompEfforts( ...
                                                posSwing(i,:), ...
                                                posSwing(i,:), ...
                                                velSwing(i,:), ...
                                                accSwing(i,:) );

        legFK = kin.leg{leg}.getFK( 'endeffector', posSwing(i,:) );
        xyzSwing(:,i) = legFK(1:3,4);

        effSwing(i,:) = gravCompEfforts + dynamicCompEfforts;
    end
end
    

time = [time time+stepPeriod];
pos = [posStance; posSwing];
vel = [velStance; velSwing];
acc = [accStance; accSwing];
eff = [effStance; effSwing];

xyzGait = [xyzStance xyzSwing];

figure;
subplot(3,1,1);
plot(time,pos);
title('Stance-Swing Trajectory');
ylabel('position (rad)');
xlabel('time (sec)');
xlim([time(1) time(end)]);
grid on;
legend(params.jointNames);

subplot(3,1,2);
plot(time,vel);
ylabel('velocity (rad/sec)');
xlabel('time (sec)');
ylim([-4 4]);
xlim([time(1) time(end)]);
grid on;

subplot(3,1,3);
plot(time,eff);
ylabel('effort (Nm)');
xlabel('time (sec)');
xlim([time(1) time(end)]);
ylim([-40 40]);
grid on;



%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Visualize the robot walking %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear gravCompEfforts dynamicCompEfforts stanceEfforts;
numSteps = 0;
isFirstDraw = true;
tic;

if logging && ~simulate
    log = legsGroup.startLog( 'dir', [params.localDir '/logs'] );
end

gainsWarmupTime = 5.0;  % [sec]
runTimer = tic();

while true
    
    t = toc;
    tGait = t - numSteps*stepPeriod;
    while tGait > stepPeriod
        tGait = tGait - stepPeriod;
        numSteps = numSteps + 1;
    end
    
    if rem(numSteps,2)==0
        stepTraj{1} = stanceTraj{1};
        stepTraj{2} = swingTraj{2};
    else
        stepTraj{1} = swingTraj{1};
        stepTraj{2} = stanceTraj{2};
    end
    
    % Do FK / Jacobians
    for leg = 1:2
        [pos,vel,acc] = stepTraj{leg}.getState(tGait);

        legFrames{leg} = kin.leg{leg}.getFK( 'output', pos );
        legCoMs{leg} = kin.leg{leg}.getFK( 'com', pos );
        
        gravCompEfforts{leg} = kin.leg{leg}.getGravCompEfforts( pos, gravityVec);
        dynamicCompEfforts{leg} = kin.leg{leg}.getDynamicCompEfforts( pos, pos, vel, acc );

        % Jacobian to the foot in the pelvis-fixed frame
        %J{leg} = kin.leg{leg}.getJacobianEndEffector( pos );
        
        % Jacobian to the pelvis in the foot-fixed frame
        J_footFixedFoot = getJacobianFixedEndEffectorFast( kin.leg{leg}, pos );
        J{leg} = J_footFixedFoot;
        
        legXYZ = squeeze(legCoMs{leg}(1:3,4,:));
        legMasses = kin.leg{leg}.getBodyMasses;
        legCoM(:,leg) = sum( legXYZ.*repmat(legMasses',3,1), 2 ) / sum(legMasses);
        legMass(leg) = sum(legMasses);
        
        legCmdAngles(leg,:) = pos;
        legCmdAngVels(leg,:) = vel; 
    end
    
    % Adjust CoM based on chassis, leg configuration
    allCoMs = [legCoM chassisCoM];
    allMasses = [legMass chassisMass];

    robotCoM = sum(allCoMs .* repmat(allMasses,3,1), 2) / sum(allMasses);
    robotMass = sum(allMasses);  
    
    % Calculate Joint Commands
    for leg = 1:2
        
        stanceWrench = zeros(6,1);
        stanceWrench(1:3) = -9.8*robotMass * gravityVec;
        
        footFrame = legFrames{leg}(:,:,end);
        footXYZ = footFrame(1:3,4);
        
        footPerpVec = getVectorPerpendicularToLine( ...
                            footXYZ, robotCoM, gravityVec );       
        stanceWrench(4:6) = 9.8*robotMass * footPerpVec;

        stanceEfforts{leg} = J{leg}' * stanceWrench;
        
%         legEfforts{leg} = stanceEfforts{leg}' + gravCompEfforts{leg} + ...
%                             dynamicCompEfforts{leg};

        % Just do grav-comp / dynamics-comp for walking in the air.        
        legEfforts{leg} = gravCompEfforts{leg} + ...
                          dynamicCompEfforts{leg};
   
        legCmdEfforts(leg,:) = legEfforts{leg};
    end
    

    %%%%%%%%%%%%%
    % VISUALIZE %
    %%%%%%%%%%%%%
    if visualizeOn
        CoMFrame = eye(4);
        CoMFrame(1:3,4) = robotCoM;

        [ stanceEfforts{1} stanceEfforts{2} ];

        allFrames = cat(3,legFrames{1},legFrames{2},chassisFrame,CoMFrame);
        %allFrames = legFrames{1};

        framesDisplay.setFrames(allFrames);
        drawnow;

        if isFirstDraw
            hold on;
            plot3(xyzGait(1,:),xyzGait(2,:),xyzGait(3,:));
            plot3(swingXYZ(:,1,1),swingXYZ(:,2,1),swingXYZ(:,3,1),'*');
            hold off;
            isFirstDraw = false;
        end
    end

    % params.legGains.controlStrategy = [];
    
    if ~simulate
        fbk = legsGroup.getNextFeedbackFull();
        
        % Use the same 'make commands' function from the floBalancing.m,
        % but transpose some inputs since we're using API calls instead of
        % the jacobians to get the velocities and efforts.
        cmd = makeLegCmds( cmd, legIndex, ...
                           legCmdAngles, legCmdAngVels', legCmdEfforts' );
        
        % Scale the gains
        if toc(runTimer) < gainsWarmupTime
            gainScale = toc(runTimer) / gainsWarmupTime;
        else
            gainScale = 1.0;
        end
        
        cmdGains.positionKp = gainScale * legGains.positionKp;
        cmdGains.velocityKp = gainScale * legGains.velocityKp;
        cmdGains.velocityFF = gainScale * legGains.velocityFF;
        cmdGains.effortKp = gainScale * legGains.effortKp;
        cmdGains.effortFF = gainScale * legGains.effortFF;
        
        legsGroup.send(cmd, 'gains', cmdGains);
    end
    
    keys = read(kb);
    if keys.SPACE || keys.ESC
        break;
    end
    
end

if logging && ~simulate
    log = legsGroup.stopLogFull();
end

floPlotting;

