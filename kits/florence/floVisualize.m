clear *;
close all;

kb = HebiKeyboard();

twoLegBalance = true;

robotFamily = 'Florence';
controllerName = 'iPad';

[ groups, kin, params ] = setupFlorence( robotFamily, controllerName );
controllerGroup = groups.controller;

legsGroup = groups.legs;
legsGroup.setFeedbackFrequency(20);
feetGroup = groups.feet;
legsGroup.setFeedbackFrequency(20);

fbkLegs = legsGroup.getNextFeedbackFull();
fbkFeet = feetGroup.getNextFeedbackFull();
fbkFeetIO = feetGroup.getNextFeedbackIO();

legFbkFrequency = legsGroup.getFeedbackFrequency();

gravityVec = params.gravityVec;
chassisFrame = params.chassisFrame;
chassisMass = params.chassisMass;
chassisCoM = params.chassisCoM;

hip1FrameNum = params.hip1FrameNum;

for leg=1:2
    legHomeFK{leg} = kin.leg{leg}.getFK('output', zeros(1,6));
    legHip1Frame(:,:,leg) = legHomeFK{leg}(:,:,hip1FrameNum);
    
    legMasses(:,leg) = kin.leg{leg}.getBodyMasses;
    legTotalMass(leg) = sum(legMasses(:,leg));
    
    
end

allMasses = [legTotalMass chassisMass];
robotMass = sum(allMasses);

legIndex = params.legIndex;

cmd = CommandStruct();

% xyzFrameLimits = [ -0.6  0.4;
%                    -0.5  0.5;
%                    -0.9  0.2 ];
% framesDisplay = FrameDisplay( [], [], xyzFrameLimits );
framesDisplay = FrameDisplay( );
isFirstDraw = true;



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

while true
    
    legsGroup.getNextFeedback( fbkLegs );
    feetGroup.getNextFeedback( fbkFeet, fbkFeetIO );

    % Timekeeping
    t = fbkLegs.time - tStart;
    dt = t - tLast;
    dt = max( dt, 0.5 * 1/legFbkFrequency );
    dt = min( dt, 2.0 * 1/legFbkFrequency );
    tLast = t;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % FEEDBACK ORIENTATION FROM IMU %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Do a check for NaNs in the pose estimator on the modules.  Break if
    % we find any so we can debug
    quatFbk = [ fbkLegs.orientationW; 
                fbkLegs.orientationX; 
                fbkLegs.orientationY; 
                fbkLegs.orientationZ ];
    if any(isnan(quatFbk(:)))
        disp('Got NaNs in the orientation feedback for at least 1 module!');
        break;
    end
    
    imuLeg = [1 2];
    imuFbkIndex = [1 9];
    hip1Orientation = quatFbk(:,1)';
    hip1AngVel = [ fbkLegs.gyroX(imuFbkIndex);
                   fbkLegs.gyroY(imuFbkIndex);
                   fbkLegs.gyroZ(imuFbkIndex) ];
    hip1RotMat = HebiUtils.quat2rotMat( hip1Orientation );
    chassisFbkRotMat = hip1RotMat * legHip1Frame(1:3,1:3,imuLeg(1))';
    
    chassisAngVel(:,1) = legHip1Frame(1:3,1:3,imuLeg(1)) * hip1AngVel(:,1);
    chassisAngVel(:,2) = legHip1Frame(1:3,1:3,imuLeg(2)) * hip1AngVel(:,2);
    chassisAngVel = mean(chassisAngVel,2);
    
    robotPose = eye(4);
    robotPose(1:3,1:3) = chassisFbkRotMat;
    gravityVec = -robotPose(3,1:3)';

    % Do FK Based on Feedback
    for leg = 1:2
        fbkPosLeg{leg} = fbkLegs.position(legIndex{leg});
        fbkVelLeg{leg} = fbkLegs.velocity(legIndex{leg});
        
        fbkLegCoMs{leg} = kin.leg{leg}.getFK( 'com', fbkPosLeg{leg} );
        fbkLegFrames{leg} = kin.leg{leg}.getFK( 'output', fbkPosLeg{leg} );
        
        % Calcule CoM for each leg
        fbkLegXYZ = squeeze(fbkLegCoMs{leg}(1:3,4,:));     
        fbkLegCoM(:,leg) = sum( fbkLegXYZ.*repmat(legMasses(:,leg)',3,1), 2 ) ...
                                    / legTotalMass(leg); 
    end
    
    % Find CoM of the whole robot based on chassis, feedback leg configuration
    allFbkCoMs = [fbkLegCoM chassisCoM];
    fbkRobotCoM = sum(allFbkCoMs .* repmat(allMasses,3,1), 2) / sum(allMasses);
    fbkCoMFrame = eye(4);
    fbkCoMFrame(1:3,4) = fbkRobotCoM;

    % allFrames = fbkCoMFrame;
    % allFrames = cat(3,chassisFrame,fbkLegFrames{1},fbkLegFrames{2},fbkCoMFrame);
    allFrames = isDoF

    for i=1:size(allFrames,3)
        allFrames(:,:,i) = robotPose * allFrames(:,:,i);
    end

    framesDisplay.setFrames(allFrames);
    drawnow;
end