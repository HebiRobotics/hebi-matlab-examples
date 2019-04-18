% Balancing Control for Florence - Biped Kit
%
% Dave Rollinson
% Nov 2017

clear *;
close all;

kb = HebiKeyboard();

visualizeOn = false;
robotFbk = true;
sendCommands = true;
logging = true;

if robotFbk
    robotFamily = 'Florence';
else
    robotFamily = [];
end

controllerName = '_Controller';

if visualizeOn
    xyzFrameLimits = [ -0.6  0.4;
                       -0.5  0.5;
                       -0.9  0.2 ];
    framesDisplay = FrameDisplay( [], [], xyzFrameLimits );
    isFirstDraw = true;
end

[ groups, kin, params ] = setupFlorence( robotFamily, controllerName );
controllerGroup = groups.controller;

stepParams = setupCapturePointWalking();

if robotFbk
    legsGroup = groups.legs;
    feetGroup = groups.feet; 
    
    legGains = params.legGains;
    cmdGains = params.cmdGains;
    gainsWarmupTime = 5.0;  % [sec]
    
    xyzChassisGainsKp = params.xyzChassisGainsKp;
    xyzChassisGainsKd = params.xyzChassisGainsKd;
    xyzFootGainsKp = params.xyzFootGainsKp;
    xyzFootGainsKd = params.xyzFootGainsKd;
    
    cmd = CommandStruct();
end    
    
numLegs = 2;
legSign = [1 -1];

gravityVec = params.gravityVec;
chassisFrame = params.chassisFrame;
chassisMass = params.chassisMass;
chassisCoM = params.chassisCoM;

springParams = params.springParams;

fbkRobotCoM = [0; 0; 0];
cmdRobotCoM = [0; 0; 0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Kinematic / Dynamic Info %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for leg=1:numLegs
    legHomeFK{leg} = kin.leg{leg}.getFK('output', zeros(1,6));
    legHip1Frame(:,:,leg) = legHomeFK{leg}(:,:,2);
    
    legMasses(:,leg) = kin.leg{leg}.getBodyMasses;
    legTotalMass(leg) = sum(legMasses(:,leg));
end

allMasses = [legTotalMass chassisMass];
robotMass = sum(allMasses);

legIndex = params.legIndex;

homeAngles = params.homeAngles;
legCmdAngles = homeAngles;

stanceHeight = params.stanceHeight;    % [m]
stanceWidth = params.stanceWidth;      % [m]
stanceMidX = params.stanceMidX;        % [m]
stanceShift = [];  % [m]

homeStanceXYZ = [  stanceMidX,     stanceMidX;
                  stanceWidth/2, -stanceWidth/2;
                 -stanceHeight,  -stanceHeight  ];
stanceXYZ = homeStanceXYZ;
stanceRot(:,:) = eye(3);

%%%%%%%%%%%%%%%%%%%
% Trajectory Info %
%%%%%%%%%%%%%%%%%%%
chassisTrajGen = params.chassisTrajGen;
rampTime = params.rampTime;

% Replan Trajectory for the mobile base
chassisTrajTime = [0 rampTime];

% Initialize trajectory for Omnibase
velocities = zeros(2,6);
accelerations = zeros(2,6);
jerks = zeros(2,6);

chassisTraj = chassisTrajGen.newJointMove( velocities, ...
                                'Velocities', accelerations, ...
                                'Accelerations', jerks, ...
                                'Time', chassisTrajTime );


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Internal history variables %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
timeHist = nan(0,1);
chassisCmdAccelHist = nan(0,6);
legStanceWeightsHist = nan(0,2);
  
if logging
    log = legsGroup.startLog( 'dir', [params.localDir '/logs'] );
end

fbk = legsGroup.getNextFeedbackFull();
tStart = fbk.time;
tLast = fbk.time;
chassisTrajStartTime = tStart;

% Get the initial feedback objects that we'll reuse later for the
% controller group.
fbkControllerIO = controllerGroup.getNextFeedbackIO();
fbkControllerMobile = controllerGroup.getNextFeedbackMobile();
latestControllerIO = fbkControllerIO;
latestControllerMobile = fbkControllerMobile;

while true
    
    fbk = legsGroup.getNextFeedbackFull();
       
    % Timekeeping
    t = fbk.time - tStart;
    dt = t - tLast;
    tLast = t;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % FEEDBACK FROM CONTROLLER INPUT %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Get feedback with a timeout of 0, which means that it returns
    % instantly, but if there was no new feedback, it returns empty.
    % This is because a mobile device on wireless and may drop out or be 
    % significantly delayed, in which case we would rather keep running 
    % with old feedback instead of waiting for new feedback.
    tempFbk = controllerGroup.getNextFeedback( ...
                    fbkControllerIO, fbkControllerMobile, 'timeout', 0 );
    if ~isempty(tempFbk)
        latestontrollerMobile = fbkControllerMobile;
        latestControllerIO = fbkControllerIO;
    end
    
    stanceTransGain = -0.2;
    stanceVelCmdXYZ = zeros(1,3);
    stanceVelCmdXYZ(2) = stanceTransGain * latestControllerIO.a1;
    stanceVelCmdXYZ(3) = stanceTransGain * latestControllerIO.a3;
    
    stanceRotGain = -0.3;
    stanceVelCmdRot = zeros(1,3);
    stanceVelCmdRot(1) = stanceRotGain * latestControllerIO.a7;
    stanceVelCmdRot(2) = stanceRotGain * latestControllerIO.a8;
    % stanceVelCmdRot(3) = stanceRotGain * latestControllerIO.a1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Evaluate Trajectory State %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Get the smoothed Chassis Velocities
    tTraj = min(t - chassisTrajStartTime, chassisTraj.getDuration);
    [chassisCmd.vel, chassisCmd.accel, chassisCmd.jerk] = ...
        chassisTraj.getState( tTraj );
    
    chassisCmdAccelHist(end+1,:) = chassisCmd.accel';
    
    stanceXYZ = stanceXYZ + chassisCmd.vel(1:3)' * dt;
    
    stanceRotAdjust = R_z(chassisCmd.vel(6)*dt) * ...
                      R_y(chassisCmd.vel(5)*dt) * ...
                      R_x(chassisCmd.vel(4)*dt);
                  
    stanceXYZ = stanceRotAdjust * stanceXYZ; 
    stanceRot = stanceRotAdjust * stanceRot;
    
    % Set up new Chassis Velocities for the trajectory generator
    chassisDesired.vel = [stanceVelCmdXYZ stanceVelCmdRot];

    velocities = [chassisCmd.vel; chassisDesired.vel];
    accelerations = [chassisCmd.accel; zeros(1,6) ];
    jerks = [chassisCmd.jerk; zeros(1,6) ];

    chassisTraj = chassisTrajGen.newJointMove( velocities, ...
                            'Velocities', accelerations, ...
                            'Accelerations', jerks, ...
                            'Time', chassisTrajTime );
    chassisTrajStartTime = t;
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % FEEDBACK ORIENTATION FROM IMU %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    imuLeg = [1 2];
    imuFbkIndex = [1 9];
    hip1Orientation = [ fbk.orientationW(1) ...
                        fbk.orientationX(1) ...
                        fbk.orientationY(1) ...
                        fbk.orientationZ(1) ];
    hip1AngVel = [ fbk.gyroX(imuFbkIndex);
                   fbk.gyroY(imuFbkIndex);
                   fbk.gyroZ(imuFbkIndex) ];
    hip1RotMat = HebiUtils.quat2rotMat( hip1Orientation );
    robotRotMat = hip1RotMat * legHip1Frame(1:3,1:3,imuLeg(1))';
    
    robotAngVel(:,1) = legHip1Frame(1:3,1:3,imuLeg(1)) * hip1AngVel(:,1);
    robotAngVel(:,2) = legHip1Frame(1:3,1:3,imuLeg(2)) * hip1AngVel(:,2);
    robotAngVel = mean(robotAngVel,2);
    
    robotPose = eye(4);
    robotPose(1:3,1:3) = robotRotMat;
    gravityVec = -robotPose(3,1:3)';
    
    % Do FK Based on Feedback
    for leg = 1:2
        fbkPosLeg{leg} = fbk.position(legIndex{leg});
        fbkVelLeg{leg} = fbk.velocity(legIndex{leg});
        
        fbkLegFrames{leg} = kin.leg{leg}.getFK( 'output', fbkPosLeg{leg} );
        fbkFootFrame{leg} = fbkLegFrames{leg}(:,:,end);
        fbkStanceXYZ(:,leg) = fbkFootFrame{leg}(1:3,4);
        fbkLegCoMs{leg} = kin.leg{leg}.getFK( 'com', fbkPosLeg{leg} );
        
        % Calcule CoM for each leg
        fbkLegXYZ = squeeze(fbkLegCoMs{leg}(1:3,4,:));     
        fbkLegCoM(:,leg) = sum( fbkLegXYZ.*repmat(legMasses(:,leg)',3,1), 2 ) ...
                                    / legTotalMass(leg); 
                                   
        stanceErrorXYZ(:,leg) = stanceXYZ(:,leg) - fbkStanceXYZ(:,leg);

        errorRotMat = stanceRot * fbkLegFrames{leg}(1:3,1:3,end)';
        [axis, angle] = HebiUtils.rotMat2axAng( errorRotMat );
        stanceErrorRot(:,leg) = angle * axis;
    end
    
    % Find CoM based on chassis, feedback leg configuration
    allFbkCoMs = [fbkLegCoM chassisCoM];
    fbkRobotCoM = sum(allFbkCoMs .* repmat(allMasses,3,1), 2) / sum(allMasses);
    fbkCoMFrame = eye(4);
    fbkCoMFrame(1:3,4) = fbkRobotCoM;
    
    % Find an XYZ stance position in the body frame that puts the feet
    % under the CoM w.r.t. gravity.
    fbkStanceMidPoint = mean(fbkStanceXYZ,2);
    stanceShiftLast = stanceShift;    
    stanceShift = getVectorPerpendicularToLine( ...
                          fbkStanceMidPoint, fbkRobotCoM, gravityVec );
    
    if isempty(stanceShiftLast)
        stanceShiftVel = [0; 0; 0];
    else
        stanceShiftVel = (stanceShift - stanceShiftLast) / dt + ...
                          cross(robotAngVel,fbkStanceMidPoint);
    end
                               
    legStanceDist(1) = getDistToLine( fbkStanceXYZ(:,1), fbkRobotCoM, gravityVec );
    legStanceDist(2) = getDistToLine( fbkStanceXYZ(:,2), fbkRobotCoM, gravityVec );
    legStanceWeights = 1 - (legStanceDist / sum(legStanceDist));
    
    legStanceWeightsHist(end+1,:) = legStanceWeights;
         
%     stanceXYZ = homeStanceXYZ + stanceShift;                  
%                       
%     newStanceMidPoint = stanceMidPoint + stanceShift;
%     newStanceMidPoint = stanceHeight * newStanceMidPoint/norm(newStanceMidPoint);
%     newStanceShift = newStanceMidPoint - stanceMidPoint;   
%     stanceXYZ = homeStanceXYZ + newStanceShift;
    
    % Get IK to XYZ stance positions to get commanded angles
    for leg=1:2
        legCmdAngles(leg,:) = kin.leg{leg}.getIK( ...
                                        'xyz', stanceXYZ(:,leg), ...
                                        'SO3', stanceRot, ...
                                        'initial', homeAngles(leg,:) );
        J_leg{leg} = kin.leg{leg}.getJacobian( 'output', ...
                                         fbkPosLeg{leg} );                            
        J_footBodyFixed{leg} = J_leg{leg}(:,:,end);   
%         J_temp = getJacobianFixedEndEffector( ...
%                                 kin.leg{leg}, 'output', fbkPosLeg{leg} );                             
%         J_footFootFixed{leg} = J_temp(:,:,1);
        J_footFootFixed{leg} = getJacobianFixedEndEffectorFast( ...
                                           kin.leg{leg}, fbkPosLeg{leg} );
        
        legCmdAngVels(:,leg) = J_footBodyFixed{leg} \ chassisCmd.vel';   
        stanceVelError(:,leg) = J_footBodyFixed{leg} * ...
                                (legCmdAngVels(:,leg) - fbkVelLeg{leg}');  
        
        stanceFootWrench = zeros(6,1);
        stanceFootWrench(1:3) = xyzFootGainsKp(1:3) .* stanceErrorXYZ(:,leg) +...
                                xyzFootGainsKd(1:3) .* stanceVelError(1:3,leg);
        stanceFootWrench(4:6) = xyzFootGainsKp(4:6) .* stanceErrorRot(:,leg) + ...
                                xyzFootGainsKd(4:6) .* stanceVelError(4:6,leg);                   
         
        stanceWrench = zeros(6,1);
        supportForce = robotMass * 9.81 * gravityVec + ...
                       -0.3 * robotMass * chassisCmd.accel(1:3)';
        stanceWrench(1:3) = supportForce;
        stanceWrench(1:3) = stanceWrench(1:3) + ...
                            -xyzChassisGainsKp(1:3) .* stanceShift + ...
                            -xyzChassisGainsKd(1:3) .* stanceShiftVel;
                        
        footPerpVec = getVectorPerpendicularToLine( ...
                            fbkStanceMidPoint, fbkRobotCoM, gravityVec );       
        stanceWrench(4:6) = 0.5 * cross(-supportForce,footPerpVec);
        
        stanceWrench = legStanceWeights(leg) * stanceWrench;
        
        chassisWrench = zeros(6,1);
%         chassisWrench(1:3) = -xyzChassisGainsKp(1:3) .* stanceShift + ...
%                              -xyzChassisGainsKd(1:3) .* stanceShiftVel;
        chassisWrench(4:6) = ...
            -xyzChassisGainsKp(4:6) .* cross(gravityVec,-stanceRot(:,3)) + ...
            -xyzChassisGainsKd(4:6) .* robotAngVel;
        chassisWrench = legStanceWeights(leg) * chassisWrench;

        chassisWrench(1:3) = fbkFootFrame{leg}(1:3,1:3) * chassisWrench(1:3);
        chassisWrench(4:6) = fbkFootFrame{leg}(1:3,1:3) * chassisWrench(4:6);

        gravCompEfforts = kin.leg{leg}.getGravCompEfforts( fbkPosLeg{leg}, gravityVec);
        
        legCmdEfforts(:,leg) = gravCompEfforts' + ...
                               J_footBodyFixed{leg}' * stanceFootWrench + ...
                               J_footBodyFixed{leg}' * stanceWrench + ...
                               J_footFootFixed{leg}' * chassisWrench;
                           
        % Spring Assist Force
        springEfforts(:,leg) = getGasSpringAssist( fbkLegFrames{leg}, ...
                                            J_leg{leg}, springParams );
        springEfforts(:,leg) = springEfforts(:,leg) + ...
                                latestControllerIO.a5*springEfforts(:,leg);
        legCmdEfforts(3:4,leg) = legCmdEfforts(3:4,leg) + ...
                                        springEfforts(3:4,leg);
        
        cmdLegFrames{leg} = kin.leg{leg}.getFK( 'output', legCmdAngles(leg,:) );
        cmdLegCoMs{leg} = kin.leg{leg}.getFK( 'com', legCmdAngles(leg,:) );
        
        % Calcule CoM for each leg
        cmdLegXYZ = squeeze(cmdLegCoMs{leg}(1:3,4,:));   
        cmdLegCoM(:,leg) = sum( cmdLegXYZ.*repmat(legMasses(:,leg)',3,1), 2 ) ...
                                    / legTotalMass(leg);                               
    end
    
    % Find CoM based on chassis, feedback leg configuration
    allCmdCoMs = [cmdLegCoM chassisCoM];
    cmdRobotCoM = sum(allCmdCoMs .* repmat(allMasses,3,1), 2) / sum(allMasses);
    cmdRobotCoM = cmdRobotCoM;
    
    cmdCoMFrame = eye(4);
    cmdCoMFrame(1:3,4) = cmdRobotCoM;
    
    cmdStanceMidPoint = mean(stanceXYZ,2);
    cmdStanceShift = getVectorPerpendicularToLine( ...
                          cmdStanceMidPoint, cmdRobotCoM, stanceRot(:,3) );
    
    % Update X stance position based on the commanded center of mass
    alpha = 0.05;
    stanceXYZ(1,:) = stanceXYZ(1,:) + alpha * cmdStanceShift(1);
    % stanceXYZ(1,:) = (1-alpha)*stanceXYZ(1,:) + alpha*cmdRobotCoM(1,:);

    if sendCommands
        cmd = makeLegCmds( cmd, legIndex, ...
                           legCmdAngles, legCmdAngVels, legCmdEfforts );
        
        % Scale the gains
        if t < gainsWarmupTime
            gainScale = t / gainsWarmupTime;
        else
            gainScale = 1.0;
        end
        
        cmdGains.positionKp = 0.5* gainScale * legGains.positionKp; 
        cmdGains.velocityKp = 2.0 * gainScale * legGains.velocityKp;
        cmdGains.velocityFF = gainScale * legGains.velocityFF;
        cmdGains.effortKp = gainScale * legGains.effortKp;
        cmdGains.effortFF = gainScale * legGains.effortFF;
        
        legsGroup.send(cmd, 'gains', cmdGains);
    end


    %%%%%%%%%%%%%
    % VISUALIZE %
    %%%%%%%%%%%%%
    if visualizeOn
        % allFrames = fbkCoMFrame;
        allFrames = cat(3,chassisFrame,fbkLegFrames{1},fbkLegFrames{2},fbkCoMFrame);

        for i=1:size(allFrames,3)
            allFrames(:,:,i) = robotPose * allFrames(:,:,i);
        end

        framesDisplay.setFrames(allFrames);
        drawnow;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%
    % CHECK FOR QUIT COMMAND %
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    keys = read(kb);
    if keys.SPACE || fbkControllerIO.b1
        break;
    end
    
    
    % STATE HISTORY
    timeHist(end+1) = t;
    %angHist(end+1,:) = reshape(legAnglesCmd',1,[]);
    %angVelHist(end+1,:) = reshape(legAngVelsCmd',1,[]);
    %effortHist(end+1,:) = reshape(legEfforts',1,[]);
end

%%
if logging
    log = legsGroup.stopLogFull();
end

%%
% Plotting
if logging
    figureSeries = 100;

    HebiUtils.plotLogs(log,'position','modules',legIndex{1},'figNum',figureSeries+11);
    %legend(moduleNames{legIndex{1}});
    HebiUtils.plotLogs(log,'position','modules',legIndex{2},'figNum',figureSeries+12);
    %legend(moduleNames{legIndex{2}});

    HebiUtils.plotLogs(log,'velocity','modules',legIndex{1},'figNum',figureSeries+21);
    %legend(moduleNames{legIndex{1}});
    HebiUtils.plotLogs(log,'velocity','modules',legIndex{2},'figNum',figureSeries+22);
    %legend(moduleNames{legIndex{2}});

    HebiUtils.plotLogs(log,'effort','modules',legIndex{1},'figNum',figureSeries+31);
    %legend(moduleNames{legIndex{1}});
    HebiUtils.plotLogs(log,'effort','modules',legIndex{2},'figNum',figureSeries+32);
    %legend(moduleNames{legIndex{2}});

    % floPlotting;
end




