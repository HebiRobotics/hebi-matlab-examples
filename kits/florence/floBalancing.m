% Balancing Control for Florence - Biped Kit
%
% Dave Rollinson
% Nov 2017

clear *;
close all;

kb = HebiKeyboard();

visualizeOn = false;
simulateFlag = false;
robotFbk = true;
sendCommands = true;
logging = true;

twoLegBalance = true;

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

[ groups, kin, params ] = setupFlorence( robotFamily, controllerName, simulateFlag );
controllerGroup = groups.controller;

if robotFbk
    legsGroup = groups.legs;
    feetGroup = groups.feet; 
    
    fbkLegs = legsGroup.getNextFeedbackFull();
    fbkFeet = feetGroup.getNextFeedbackFull();
    fbkFeetIO = feetGroup.getNextFeedbackIO();
    
    legFbkFrequency = legsGroup.getFeedbackFrequency();
    
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
bungeeParams = params.bungeeParams;

fbkRobotCoM = [0; 0; 0];
cmdRobotCoM = [0; 0; 0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Kinematic / Dynamic Info %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Number for where the input to hip1 is in the kinematic chain
hip1FrameNum = params.hip1FrameNum;

for leg=1:numLegs
    legHomeFK{leg} = kin.leg{leg}.getFK('output', zeros(1,6));
    legHip1Frame(:,:,leg) = legHomeFK{leg}(:,:,hip1FrameNum);
    
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

% % Shift to be over one foot
% homeStanceXYZ(2,:) = homeStanceXYZ(2,:) + stanceWidth/2;
             
stanceCmdXYZ = homeStanceXYZ;
stanceCmdRot = eye(3);

stanceAdjustXYZ = zeros(3,2);
stanceShiftXYZ = zeros(3,2);
stanceRPY = zeros(1,3);

footRot = params.footRot;

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
footIkWrenchHist = nan(6,2,0);
stanceWrench1Hist = nan(6,2,0);
stanceWrench2Hist = nan(6,2,0);
chassisWrenchHist = nan(6,2,0);
stanceCmdXYZHist = nan(3,2,0);
  
if logging
    log = legsGroup.startLog( 'dir', [params.localDir '/logs'] );
    % feetLog = feetGroup.startLog( 'dir', [params.localDir '/logs'] );
end

fbkLegs = legsGroup.getNextFeedbackFull();
tStart = fbkLegs.time;
tLast = fbkLegs.time;
chassisTrajStartTime = tStart;

% Get the initial feedback objects that we'll reuse later for the
% controller group.
fbkControllerIO = controllerGroup.getNextFeedbackIO();
fbkControllerMobile = controllerGroup.getNextFeedbackMobile();
latestControllerIO = fbkControllerIO;
latestControllerMobile = fbkControllerMobile;

twoLegBalanceWeight = 1;

while true
    
    legsGroup.getNextFeedback( fbkLegs );
    feetGroup.getNextFeedback( fbkFeet, fbkFeetIO );

    % Timekeeping
    t = fbkLegs.time - tStart;
    dt = t - tLast;
    dt = max( dt, 0.5 * 1/legFbkFrequency );
    dt = min( dt, 2.0 * 1/legFbkFrequency );
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
    
    stanceTransGain = -0.3;
    stanceVelCmdXYZ = zeros(1,3);
    % stanceVelCmdRot(1) = 0;                                     % X - Chassis Fwd-Back
    stanceVelCmdXYZ(2) = stanceTransGain * latestControllerIO.a1; % Y - Chassis Left-Right
    stanceVelCmdXYZ(3) = stanceTransGain * latestControllerIO.a3; % Z - Chassis Height
    
    stanceRotGain = -0.5;
    stanceVelCmdRot = zeros(1,3);
    % stanceVelCmdRot(1) = 0;                                    % Chassis Roll
    stanceVelCmdRot(2) = stanceRotGain * latestControllerIO.a8;  % Chassis Pitch
    stanceVelCmdRot(3) = stanceRotGain * latestControllerIO.a7;  % Chassis Yaw
    
    switchLowPass = .98;
    twoLegBalance = ~latestControllerIO.b6;
    twoLegBalanceWeight = switchLowPass * twoLegBalanceWeight + ...
                          (1-switchLowPass) * twoLegBalance;
                      
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Evaluate Trajectory State %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Get the smoothed Chassis Velocities
    tTraj = min(t - chassisTrajStartTime, chassisTraj.getDuration);
    [chassisCmd.vel, chassisCmd.accel, chassisCmd.jerk] = ...
        chassisTraj.getState( tTraj );
    
    chassisCmdAccelHist(end+1,:) = chassisCmd.accel';
    
    stanceShiftXYZ = stanceShiftXYZ + chassisCmd.vel(1:3)' * dt;
    
    
    stanceRPY(1) = stanceRPY(1) + chassisCmd.vel(4)*dt;
    stanceRPY(2) = stanceRPY(2) + chassisCmd.vel(5)*dt;
    stanceRPY(3) = stanceRPY(3) + chassisCmd.vel(6)*dt;
    
    stanceCmdRot = R_z(stanceRPY(3)) * ...
                   R_y(stanceRPY(2)) * ...
                   R_x(stanceRPY(1));
    
    stanceCmdXYZ = stanceCmdRot * (homeStanceXYZ + stanceShiftXYZ + stanceAdjustXYZ);              
    
    
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
    
    % Do a check for NaNs in the pose estimator on the modules.  Break if
    % we find any so we can debug
    quatFbk = [ fbkLegs.orientationW; 
                fbkLegs.orientationX; 
                fbkLegs.orientationY; 
                fbkLegs.orientationZ ];
    nanFbkModules = isnan(quatFbk(1,:));
    if any(nanFbkModules)
        disp('Got NaNs in the orientation feedback for at least 1 module.');
        disp('Please restart:');
        disp(legGroup.getInfo.name(nanFbkModules));
        break;
    end
    
    imuLeg = [1 2];
    imuFbkIndex = [1 9];
    hip1Orientation = quatFbk(:,imuFbkIndex)';
    hip1AngVel = [ fbkLegs.gyroX(imuFbkIndex);
                   fbkLegs.gyroY(imuFbkIndex);
                   fbkLegs.gyroZ(imuFbkIndex) ];
    hip1RotMat = HebiUtils.quat2rotMat( hip1Orientation );
    chassisFbkRotMat(:,:,1) = hip1RotMat(:,:,1) * legHip1Frame(1:3,1:3,imuLeg(1))';
    chassisFbkRotMat(:,:,2) = hip1RotMat(:,:,2) * legHip1Frame(1:3,1:3,imuLeg(2))';
    
    chassisAngVel(:,1) = legHip1Frame(1:3,1:3,imuLeg(1)) * hip1AngVel(:,1);
    chassisAngVel(:,2) = legHip1Frame(1:3,1:3,imuLeg(2)) * hip1AngVel(:,2);
    chassisAngVel = mean(chassisAngVel,2);
    
    chassisAngVelError = chassisCmd.vel(4:6)' - chassisAngVel;
    
    robotPose = eye(4);
    robotPose(1:3,1:3) = chassisFbkRotMat(:,:,1);  % Use just left leg for pose
    gravityVec = mean( -squeeze(chassisFbkRotMat(3,:,:)), 2 ); % Use both for gravity vec
    gravityVec = gravityVec / norm(gravityVec);
    
    feetRotVel = [ fbkFeet.gyroX;
                   fbkFeet.gyroY;
                   fbkFeet.gyroZ ];    
               
    feetRotVelError = -feetRotVel;
    
    footIkWrench = zeros(6,2);
    chassisWrench = zeros(6,2);
    stanceWrench = zeros(6,2);
    
    % Do FK Based on Feedback, get Jacobians, and other stuff that is
    % common having one or both feet on the ground.
    for leg = 1:2
        fbkPosLeg{leg} = fbkLegs.position(legIndex{leg});
        fbkVelLeg{leg} = fbkLegs.velocity(legIndex{leg});
        
        fbkLegFrames{leg} = kin.leg{leg}.getFK( 'output', fbkPosLeg{leg} );
        fbkFootFrame{leg} = fbkLegFrames{leg}(:,:,end);
        fbkStanceXYZ(:,leg) = fbkFootFrame{leg}(1:3,4);
        fbkLegCoMs{leg} = kin.leg{leg}.getFK( 'com', fbkPosLeg{leg} );
        
        % Get Jacobians for each leg in the body frame
        J_legBodyFixed{leg} = kin.leg{leg}.getJacobian( 'output', ...
                                         fbkPosLeg{leg} );                            
        J_footBodyFixed{leg} = J_legBodyFixed{leg}(:,:,end);   
        
        % Also get Jacobians for each leg in the foot frame
        J_legFootFixed{leg} =  kin.leg{leg}.getJacobian( 'output', ...
                                         fbkPosLeg{leg}, 'fixedFrame', -1 );
        J_footFootFixed{leg} =  J_legFootFixed{leg}(:,:,1);   
                                              
        % Calcule CoM for each leg
        fbkLegXYZ = squeeze(fbkLegCoMs{leg}(1:3,4,:));     
        fbkLegCoM(:,leg) = sum( fbkLegXYZ.*repmat(legMasses(:,leg)',3,1), 2 ) ...
                                    / legTotalMass(leg); 
                                   
        stanceErrorXYZ(:,leg) = stanceCmdXYZ(:,leg) - fbkStanceXYZ(:,leg);

        errorRotMat = stanceCmdRot * fbkLegFrames{leg}(1:3,1:3,end)';
        [axis, angle] = HebiUtils.rotMat2axAng( errorRotMat );
        stanceErrorRot(:,leg) = angle * axis;
        
        legCmdAngVels(:,leg) = J_footBodyFixed{leg} \ chassisCmd.vel';   
        stanceVelError(:,leg) = J_footBodyFixed{leg} * ...
                                (legCmdAngVels(:,leg) - fbkVelLeg{leg}');  
                            
        footIkWrench(1:3,leg) = xyzFootGainsKp(1:3) .* stanceErrorXYZ(:,leg) +...
                                xyzFootGainsKd(1:3) .* stanceVelError(1:3,leg);
%         footIkWrench(4:6,leg) = xyzFootGainsKp(4:6) .* stanceErrorRot(:,leg) + ...
%                                 xyzFootGainsKd(4:6) .* stanceVelError(4:6,leg);
        footIkWrench(4:6,leg) = xyzFootGainsKp(4:6) .* stanceErrorRot(:,leg) + ...
                                xyzFootGainsKd(4:6) .* feetRotVelError(:,leg);                                          
        gravCompEfforts(leg,:) = kin.leg{leg}.getGravCompEfforts( fbkPosLeg{leg}, gravityVec);
    end
    
    % Find CoM of the whole robot based on chassis, feedback leg configuration
    allFbkCoMs = [fbkLegCoM chassisCoM];
    fbkRobotCoM = sum(allFbkCoMs .* repmat(allMasses,3,1), 2) / sum(allMasses);
    fbkCoMFrame = eye(4);
    fbkCoMFrame(1:3,4) = fbkRobotCoM;
    
    % Find CoM of just the robot chassis and unsupported leg 
    oneLegStanceWeights = [0 1];
    swingMasses = allMasses;
    swingMasses(1:2) = allMasses(1:2) .* (1 - oneLegStanceWeights);
    swingRobotCoM = sum(allFbkCoMs .* repmat(swingMasses,3,1), 2) / sum(swingMasses);
    swingMass = sum(swingMasses);

    %%%%%%%%%%%%%%%%%%%%%%%
    % Balance on two legs %
    %%%%%%%%%%%%%%%%%%%%%%%
    fbkStanceMidPoint = mean(fbkStanceXYZ,2);
    stanceShiftLast = stanceShift;    
    stanceShift = getVectorPerpendicularToLine( ...
                          fbkStanceMidPoint, fbkRobotCoM, gravityVec );

    if isempty(stanceShiftLast)
        stanceShiftVel = [0; 0; 0];
    else
        stanceShiftVel = (stanceShift - stanceShiftLast) / dt + ...
                          cross(chassisAngVel,fbkStanceMidPoint);
    end

    legStanceDist(1) = getDistToLine( fbkStanceXYZ(:,1), fbkRobotCoM, gravityVec );
    legStanceDist(2) = getDistToLine( fbkStanceXYZ(:,2), fbkRobotCoM, gravityVec );
    legStanceWeights = 1 - (legStanceDist / sum(legStanceDist));

    % Get IK to XYZ stance positions to get commanded angles
    for leg=1:2
        
        % Damp out linear motion due to rotational motion about the feet 
        chassisWrench(1:3,leg) = xyzChassisGainsKd(1:3) .* ...
                                cross(chassisAngVelError,-fbkStanceMidPoint);
        
        % Keep the chassis level w.r.t. gravity   
        chassisWrench(4:6,leg) = ...
            xyzChassisGainsKp(4:6) .* cross(gravityVec,stanceCmdRot(:,3)) + ...
            xyzChassisGainsKd(4:6) .* chassisAngVelError;

        chassisWrench(1:3,leg) = fbkFootFrame{leg}(1:3,1:3) * chassisWrench(1:3,leg);
        chassisWrench(4:6,leg) = fbkFootFrame{leg}(1:3,1:3) * chassisWrench(4:6,leg);

        supportForce = robotMass * 9.81 * gravityVec + ...
                       -0.3 * robotMass * chassisCmd.accel(1:3)';
        stanceWrench(1:3,leg) = supportForce;
        stanceWrench(1:3,leg) = stanceWrench(1:3,leg) + ...
                            -xyzChassisGainsKp(1:3) .* stanceShift + ...
                            -xyzChassisGainsKd(1:3) .* stanceShiftVel;
                        
        footPerpVec = getVectorPerpendicularToLine( ...
                            fbkStanceMidPoint, fbkRobotCoM, gravityVec );       
        stanceWrench(4:6,leg) = 0.75 * cross(-supportForce,footPerpVec);

        stanceWrench(:,leg) = legStanceWeights(leg) * stanceWrench(:,leg);
        chassisWrench(:,leg) = legStanceWeights(leg) * chassisWrench(:,leg);

        twoLegCmdEfforts(:,leg) = gravCompEfforts(leg,:)' + ...
                               J_footBodyFixed{leg}' * footIkWrench(:,leg) + ...
                               J_footBodyFixed{leg}' * stanceWrench(:,leg) + ...
                               J_footFootFixed{leg}' * chassisWrench(:,leg);               
    end
    
    footIkWrenchHist(:,:,end+1) = footIkWrench;
    stanceWrench2Hist(:,:,end+1) = stanceWrench;
    chassisWrenchHist(:,:,end+1) = chassisWrench;

    %%%%%%%%%%%%%%%%%%%%%%
    % Balance on one leg %
    %%%%%%%%%%%%%%%%%%%%%%
    stanceWrench = zeros(6,2);
    legSupportEfforts = zeros(6,2);
    
    for leg=1:2
        
        gravityVecFootFrame = fbkFootFrame{leg}(1:3,1:3) * gravityVec;
        
        for i=1:length(legMasses(:,leg))
            gravCompForce = -legMasses(i,leg) * gravityVecFootFrame;
            legSupportEfforts(:,leg) = legSupportEfforts(:,leg) + ...
                        J_legFootFixed{leg}(1:3,:,i)' * gravCompForce;
        end 

        supportForce = -swingMass * 9.81 * gravityVec + ...
                       0.3 * swingMass * chassisCmd.accel(1:3)';  
                   
        stanceWrench(1:3,leg) = supportForce;
        stanceWrench(4:6,leg) = cross(-supportForce,swingRobotCoM);    
        stanceWrench(1:3,leg) = fbkFootFrame{leg}(1:3,1:3) * ...
                                       stanceWrench(1:3,leg);
        stanceWrench(4:6,leg) = fbkFootFrame{leg}(1:3,1:3) * ...
                                       stanceWrench(4:6,leg);
        % stanceWrench(6,leg) = 0;
        
        footIkWrench(:,leg) = oneLegStanceWeights(leg) * footIkWrench(:,leg);
        stanceWrench(:,leg) = oneLegStanceWeights(leg) * stanceWrench(:,leg);
        chassisWrench(:,leg) = oneLegStanceWeights(leg) * chassisWrench(:,leg);
        
        oneLegCmdEfforts(:,leg) = legSupportEfforts(:,leg) + ...
                   J_footBodyFixed{leg}' * footIkWrench(:,leg) + ...
                   J_footFootFixed{leg}' * stanceWrench(:,leg) + ...
                   J_footFootFixed{leg}' * chassisWrench(:,leg);           
    end
    
    stanceWrench1Hist(:,:,end+1) = stanceWrench;

    legCmdEfforts = twoLegBalanceWeight * twoLegCmdEfforts + ...
                    (1-twoLegBalanceWeight) * oneLegCmdEfforts;
    
%     stanceXYZ(3,1) = twoLegBalanceWeight*homeStanceXYZ(3,1) + ...
%                         (1-twoLegBalanceWeight)*(homeStanceXYZ(3,1)+0.2);
                
    
    %%%%%%%%%%%%%%%%%%
    % COMMON TO BOTH %
    %%%%%%%%%%%%%%%%%%
    for leg=1:2
        % Spring Assist Force
        springEfforts(:,leg) = getGasSpringAssist( fbkLegFrames{leg}, ...
                                            J_legBodyFixed{leg}, springParams );
%         springEfforts(:,leg) = springEfforts(:,leg) + ...
%                                 latestControllerIO.a5*springEfforts(:,leg);
        legCmdEfforts(3:4,leg) = legCmdEfforts(3:4,leg) + ...
                                        springEfforts(3:4,leg);
                                    
        % Bungee cord mod 
        bungeeEffort = bungeeParams.baseForce * bungeeParams.momentArm;
        legCmdEfforts(2,leg) = legCmdEfforts(2,leg) + ...
                                   legSign(leg) * bungeeEffort;
        
        legCmdAngles(leg,:) = kin.leg{leg}.getIK( ...
                                    'xyz', stanceCmdXYZ(:,leg), ...
                                    'SO3', stanceCmdRot * footRot(:,:,leg), ...
                                    'initial', homeAngles(leg,:) ); 

        cmdLegFrames{leg} = kin.leg{leg}.getFK( 'output', legCmdAngles(leg,:) );
        cmdLegCoMs{leg} = kin.leg{leg}.getFK( 'com', legCmdAngles(leg,:) );

        % Calcule CoM for each leg
        cmdLegXYZ = squeeze(cmdLegCoMs{leg}(1:3,4,:));   
        cmdLegCoM(:,leg) = sum( cmdLegXYZ.*repmat(legMasses(:,leg)',3,1), 2 ) ...
                                    / legTotalMass(leg);  
    end
    
    springEfforts(3:4,:);
    
    % Find CoM based on chassis, commanded leg configuration
    allCmdCoMs = [cmdLegCoM chassisCoM];
    cmdRobotCoM = sum(allCmdCoMs .* repmat(allMasses,3,1), 2) / sum(allMasses);
    cmdRobotCoM = cmdRobotCoM;
    
    cmdCoMFrame = eye(4);
    cmdCoMFrame(1:3,4) = cmdRobotCoM;
    
    cmdStanceMidPoint = mean(stanceCmdXYZ,2);
    cmdStanceShift = getVectorPerpendicularToLine( ...
                          cmdStanceMidPoint, cmdRobotCoM, stanceCmdRot(:,3) );
    cmdStanceShift(2:3) = 0;

    % Update X stance position based on the commanded center of mass
    alpha = 0.1;
    stanceAdjustXYZ = stanceAdjustXYZ + alpha * cmdStanceShift;
    
    if sendCommands
        cmd = makeLegCmds( cmd, legIndex, ...
                           legCmdAngles, legCmdAngVels, legCmdEfforts );
        
        % Scale the gains
        if t < gainsWarmupTime
            gainScale = (t / gainsWarmupTime)^2;
        else
            gainScale = 1.0;
        end
        
        cmdGains.positionKp = 1.0 * gainScale * legGains.positionKp; 
        cmdGains.velocityKp = 1.0 * gainScale * legGains.velocityKp;
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

    % STATE HISTORY
    timeHist(end+1) = t;
    legStanceWeightsHist(end+1,:) = legStanceWeights;
    stanceCmdXYZHist(:,:,end+1) = stanceCmdXYZ;
         
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    % CHECK FOR QUIT COMMAND %
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    keys = read(kb);
    if keys.SPACE || fbkControllerIO.b1
        
        % Set the button configs, using acknowledgements.
        numSends = 0;
        maxSends = 10;
        ack = false;
        fprintf('Quitting.\n');
        while ~ack
            cmdIO = IoCommandStruct();
            % Reset the screen back to normal
            cmdIO.a3 = nan; 
            cmdIO.e1 = 0;  
            cmdIO.b6 = 0;   
            ack = controllerGroup.send( cmdIO, ...
                                        'led', [], ...
                                        'ack', true );
            numSends = numSends + 1;
            if numSends > maxSends
                % disp('Did not receive acknowledgement from controller.');
                break;
            end
            
            
        end
        break;
    end
    
end

%%
if logging
    log = legsGroup.stopLogFull();
    feetLog = feetGroup.stopLogFull();
end

%%
% Plotting
if logging
    floPlotting;
end




