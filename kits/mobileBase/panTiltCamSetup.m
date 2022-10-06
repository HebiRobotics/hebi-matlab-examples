function [panTiltArm, params] = panTiltCamSetup(robotFamily, cameraName )

%Module names
params.panTiltNames = HebiLookup.newGroupFromNames(robotFamily, ...
    {'C1_pan', 'C2_tilt'});

%load and send gains

%Kinematic modeul
camKin = HebiKinematics('hrdf/Chevron-CamTail');

% API Wrappers
panTilt = HebiArm(group, kin);

% Default seed positions for doing inverse kinematics
params.ikSeedPos = [0 0];

% Trajectory generator parameters
params.minTrajDuration = 0.33; % [sec]
params.defaultSpeedFactor = 0.9;
arm.trajGen.setMinDuration(params.minTrajDuration);
arm.trajGen.setSpeedFactor(params.defaultSpeedFactor);

end

