function [panTiltArm, params] = setupPanTiltArm(robotFamily, cameraName )

%Module names
params.names = {'C1_pan', 'C2_tilt'};
group = HebiLookup.newGroupFromNames(robotFamily, params.names);

%load and send gains

%Kinematic modeul
camKin = HebiKinematics('hrdf/Chevron-CamTail');


% API Wrappers
panTiltArm = HebiArm(group, camKin);

% Default seed positions for doing inverse kinematics
params.ikSeedPos = [0 0];

% Trajectory generator parameters
params.minTrajDuration = 0.33; % [sec]
params.defaultSpeedFactor = 0.9;
panTiltArm.trajGen.setMinDuration(params.minTrajDuration);
panTiltArm.trajGen.setSpeedFactor(params.defaultSpeedFactor);

end

