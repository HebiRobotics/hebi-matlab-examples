% Prototype Hexapod Kinematics
%
% Dave Rollinson
% Feb 2017

clear *;
close all;
HebiLookup.initialize();

localDir = fileparts( mfilename('fullpath') );

% Optional flags
visualizeOn = false;
simulate = false;
logging = true;

[legKin, chassisKin] = makeDaisyKinematics();

for leg=1:length(legKin)
    trajGen{leg} = HebiTrajectoryGenerator(legKin{leg}); 
end

robotName = 'Daisy';
moduleNames = { 'base1', 'shoulder1', 'elbow1', ...
                'base2', 'shoulder2', 'elbow2', ...
                'base3', 'shoulder3', 'elbow3', ...
                'base4', 'shoulder4', 'elbow4', ...
                'base5', 'shoulder5', 'elbow5', ...
                'base6', 'shoulder6', 'elbow6' };

setDaisyParams;

% Approx home leg positions 
seedAngles = repmat( [-0.1 -0.3 -1.9], numLegs, 1 );
seedAngles([2 4 6],:) = -seedAngles([2 4 6],:);

if simulate
    tempAngles = seedAngles';
    fbk.position = tempAngles(:)';
else
    legsGroup = HebiLookup.newGroupFromNames( robotName, moduleNames );
    legsGroup.setFeedbackFrequency(200);
    pause(0.2);
    fbk = legsGroup.getNextFeedback();

    % Load gains for the legs and replicate them 6X to set for the whole robot
    legGains = HebiUtils.loadGains( [localDir '/gains/daisyLeg-Gains.xml'] );
    gainFields = fields( legGains );
    for i=1:length(gainFields)                    
        legGains.(gainFields{i}) = repmat( legGains.(gainFields{i}), [1 6] );
    end
    
    % Set the gains, using acknowledgements.
    numSends = 0;
    maxSends = 20;
    ack = false;
    while ~ack
        ack = legsGroup.send( 'gains', legGains, 'ack', true );
        numSends = numSends + 1;
        if numSends > maxSends
            error('Could not receive acknowledgement from at least 1 module');
        end
    end
end

% Feedback for initial stance position
for leg=1:numLegs
    legEndPointFrame = legKin{leg}.getFK( 'endeffector', ...
                           fbk.position(jointInds(leg,:)) ); 
    fbkStanceXYZ(:,leg) = legEndPointFrame(1:3,4); 
end
stanceXYZ = fbkStanceXYZ;



if visualizeOn
    xyzFrameLimits = [ -0.75  0.75;
                       -0.75  0.75;
                       -0.75  0.25 ];
    framesDisplay = FrameDisplay( [], [], xyzFrameLimits );
end

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SETUP THE MOBILE I/O CONTROLLER %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
controllerName = '_controller';
controllerGroup = setupDaisyController( robotName, controllerName );

% Get the initial feedback objects that we'll reuse later for the
% controller group.
fbkControllerIO = controllerGroup.getNextFeedbackIO();
fbkControllerMobile = controllerGroup.getNextFeedbackMobile();
latestControllerIO = fbkControllerIO;
latestControllerMobile = fbkControllerMobile;

if logging 
    legsGroup.startLog('dir',[localDir '/logs']);
end

mainTime = tic;
tocLast = toc(mainTime);
while true
    
    if ~simulate
        fbk = legsGroup.getNextFeedbackFull();
    end
    
    % Timekeeping
    tocNow = toc(mainTime);
    dt = tocNow - tocLast;
    tocLast = tocNow;
    
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
    [xyzVel, rotVel, auxCmd] = getJoyCommands( latestControllerIO );
    
    
    % Use the 6 IMUs around the base to get the orientation.  Rotate them
    % into the body frame, convert to euler angles, and then average the
    % pitch and roll to remove the drifting yaw component.
    imuModules = 1:3:18;
    T = eye(4);
    
    Q = [ fbk.orientationW;
          fbk.orientationX;
          fbk.orientationY;
          fbk.orientationZ ];
    
    if ~simulate
        
        for i=1:length(imuModules)
            j = imuModules(i);
            imuFrame = legKin{i}.getBaseFrame();
            
            DCM_module = HebiUtils.quat2rotMat(Q(:,j)');
            DCM_module = DCM_module * imuFrame(1:3,1:3)';
            try
                RPY = SpinCalc('DCMtoEA123',DCM_module,1E-9,0);
            catch
                % fprintf('Module %d is near singularity.\n', i);
                % keyboard
                RPY = [nan nan nan];
            end
            
            RPY(RPY>180) =  RPY(RPY>180) - 360;           
            RPY_module(i,:) = -RPY;
        end

        rollAngle = mean(RPY_module(:,1),'omitnan');
        pitchAngle = mean(RPY_module(:,2),'omitnan');
        
        rollR = R_x(deg2rad(rollAngle));
        pitchR = R_y(deg2rad(pitchAngle));
        
        T(1:3,1:3) = pitchR * rollR;
        % T = eye(4);
    else
        T = eye(4);
    end
    
    if ~isempty(auxCmd) && auxCmd.quit
          
        % Set the button configs, using acknowledgements.
        numSends = 0;
        maxSends = 2;
        ack = false;
        fprintf('Quitting.\n');
        while ~ack
            ack = controllerGroup.send( 'led', [], ...
                                        'ack', true );
            numSends = numSends + 1;
            if numSends > maxSends
                % disp('Did not receive acknowledgement from controller.');
                break;
            end
        end
        
        break;
    end
    
    if stepMode ~= auxCmd.steppingMode
        stepMode = auxCmd.steppingMode;
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
        legEndPointFrame = legKin{leg}.getFK( 'endeffector', ...
                               fbk.position(jointInds(leg,:)) ); 
        fbkStanceXYZ(:,leg) = legEndPointFrame(1:3,4); 
    end
    
    % Make the home stance match the current Z-height
    levelHomeStanceXYZ(3,:) = levelHomeStanceXYZ(3,:) + xyzVel(3)*dt;
    homeStanceXYZ = levelHomeStanceXYZ;
    
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
        
        if rem(leg,2)==1
            legSign = -1;
        else
            legSign = 1;
        end
        
        %seedAngles(leg,:) = legAngles(leg,:);   % Last commands
        seedAngles(leg,:) = fbk.position(jointInds(leg,:)); % Latest feedback

        if isStance(leg)
            legAngles(leg,:) = legKin{leg}.getIK( 'xyz', stanceXYZ(:,leg), ...
                                     'initial', seedAngles(leg,:) );
            dynamicCompEffort = zeros(1,3);                  
        else
            % REPLAN SWING EVERY TIME
            stepWaypoints{leg} = [ liftUpXYZ(:,leg)';
                                   midStepXYZ(:,leg)';
                                   touchDownXYZ(:,leg)' ];
                               
            for j=1:size(stepWaypoints{leg},1)             
                stepAngles{leg}(j,:) = legKin{leg}.getIK( ...
                                     'xyz', stepWaypoints{leg}(j,:), ...
                                     'initial', seedAngles(leg,:) );
            end
            
            if tocSwing==0
                % Starting Step Vel/Accel based on liftOff Velocity
                stepAngVels{leg} = nan(size(stepAngles{leg}));
                J = legKin{leg}.getJacobianEndEffector( stepAngles{leg}(1,:) );           
                stepAngVels{leg}(1,:) = J(1:3,:) \ liftOffVelXYZ(:,leg); 
                
                stepAngAccels{leg} = nan(size(stepAngles{leg}));
                stepAngAccels{leg}(1,:) = 0; 
            end
            
            % Ending Step Vel\Accel for Trajectory
            J = legKin{leg}.getJacobianEndEffector( stepAngles{leg}(end,:) );
            stepAngVels{leg}(end,:) = J(1:3,:) \ stanceVelXYZ(:,leg);
            stepAngAccels{leg}(end,:) = 0;
            
            swingTraj{leg} = trajGen{leg}.newJointMove( stepAngles{leg}, ...
                                        'time', swingTimeVec, ...
                                        'velocities', stepAngVels{leg}, ...
                                        'accelerations', stepAngAccels{leg} );  
    
            [pos,vel,acc] = swingTraj{leg}.getState(tocSwing);
            legAngles(leg,:) = pos;
            
            dynamicCompEffort = legKin{leg}.getDynamicCompEfforts( ...
                                    pos, pos, vel, acc ); 
        end              
          
        J = legKin{leg}.getJacobian('endeffector',legAngles(leg,:)); 
        J_xyz = J(1:3,:);
        
        if isStance(leg)
            legAngVels(leg,:) = J_xyz \ stanceVelXYZ(:,leg);         
        else
            legAngVels(leg,:) = vel;
        end 
        
        frames(:,:,frameIndex(:,leg)) = ...
                    legKin{leg}.getFK('output',legAngles(leg,:));

        gravCompEffort = legKin{leg}.getGravCompEfforts( legAngles(leg,:), gravityVec );
        
        springShift = 5.0; %N-m
        dragShift = 1.5; % N-m / (rad/sec)
        springEffort = [0 springShift 0] + [0 dragShift 0] .* legAngVels(leg,:);
        stanceEffort = J_xyz' * cmdFootForce(:,leg);
        legEfforts(leg,:) = gravCompEffort + stanceEffort' + ...
                            dynamicCompEffort + legSign*springEffort;
          
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
        legsGroup.set(cmd);
    end
    
    % STATE HISTORY
    timeHist(end+1) = tocNow;
    angHist(end+1,:) = reshape(legAngles',1,[]);
    angVelHist(end+1,:) = reshape(legAngVels',1,[]);
    effortHist(end+1,:) = reshape(legEfforts',1,[]);

      
    % ANIMATION
    if visualizeOn
        framesDisplay.setFrames( frames );
        drawnow;
    end        
end

if logging
    log = legsGroup.stopLogFull();
end

%% PLOTTING COMMAND HISTORY - SPACE BAR TO ADVANCE THRU LEGS
% if logging
%     figure(42);
% 
%     for plotLeg=1:numLegs
%         % Position
%         ax = subplot(3,1,1);
%         plot(timeHist,angHist(:,jointInds(plotLeg,:)),'--');
%         set(ax,'colorOrderIndex',1);
%         if isCommandLeg(plotLeg)
%             hold on;
%             plot(log.time,log.position(:,jointInds(plotLeg,:)));
%             hold off;
%         end
%         title(['Joint Angles - Leg: ' num2str(plotLeg)]);
%         xlabel('time (sec)');
%         ylabel('angles (rad)');
%         xlim([timeHist(1) timeHist(end)]);
%         ylim([-3.5 3.5]);
%         grid on;
% 
%         % Velocity
%         ax = subplot(3,1,2);
%         plot(timeHist,angVelHist(:,jointInds(plotLeg,:)),'--');
%         set(ax,'colorOrderIndex',1);
%         if isCommandLeg(plotLeg)
%             hold on;
%             plot(log.time,log.velocity(:,jointInds(plotLeg,:)));
%             hold off;
%         end
%         title(['Joint Velocities - Leg: ' num2str(plotLeg)]);
%         xlabel('time (sec)');
%         ylabel('velocity (rad/sec)');
%         xlim([timeHist(1) timeHist(end)]);
%         ylim([-4 4]);
%         grid on;
% 
%         % Effort
%         ax = subplot(3,1,3);
%         plot(timeHist,effortHist(:,jointInds(plotLeg,:)),'--');
%         set(ax,'colorOrderIndex',1);
%         if isCommandLeg(plotLeg)
%             hold on;
%             plot(log.time,log.effort(:,jointInds(plotLeg,:)));
%             hold off;
%         end
%         title(['Joint Efforts - Leg: ' num2str(plotLeg)]);
%         xlabel('time (sec)');
%         ylabel('effort (N-m)');
%         xlim([timeHist(1) timeHist(end)]); 
%         ylim([-10 15]);
%         grid on;
% 
%         pause;
%     end
% end