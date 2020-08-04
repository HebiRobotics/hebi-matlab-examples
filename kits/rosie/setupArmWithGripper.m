function [ arm, params, gripper ] = setupArmWithGripper(family)

% Arm Module Names
group = HebiLookup.newGroupFromNames(family, {
    'J1_base'
    'J2A_shoulder1'
    'J3_shoulder2'
    'J4_elbow'
    'J5_wrist1'
    'J6_wrist2'
    'J7_wrist3' });

% Load and send Gains
% REMINDER: change gains to correct file
params.gains = HebiUtils.loadGains('gains/A-2099-07G'); 
HebiUtils.sendWithRetry(group, 'gains', params.gains);

% Kinematic Model
% REMINDER: change hrdf to correct file
kin = HebiKinematics('hrdf/A-2099-07G'); 

% Gripper
gripperGroup = HebiLookup.newGroupFromNames(family, 'gripperSpool');
params.gripperGains = HebiUtils.loadGains('gains/gripper-gains');
% gripperGroup = [];
% params.gripperGains = [];
% HebiUtils.sendWithRetry(gripperGroup, 'gains', params.gripperGains);

% API Wrappers
arm = HebiArm(group, kin);
if ~isempty(gripperGroup)
    gripper = HebiGripper(gripperGroup);
else
    gripper = [];
end

% Default seed positions for doing inverse kinematics
params.ikSeedPos = [0 -1 0 2.5 1.5 -1.5 1];

% Trajectory generator parameters
params.minTrajDuration = 0.33; % [sec]
params.defaultSpeedFactor = 0.9;
arm.trajGen.setMinDuration(params.minTrajDuration);
arm.trajGen.setSpeedFactor(params.defaultSpeedFactor);

% (Optional) Compensation to joint efforts due to a gas spring (if present)
shoulderJointComp = 0; % Nm  <--- Change this if you add a gas spring
params.effortOffset = [0 shoulderJointComp 0 0 0 0 0]; 

% Default plugins
doubleShoulder = HebiLookup.newGroupFromNames(family, 'J2B_shoulder1');
arm.plugins = {
    HebiArmPlugins.EffortOffset(params.effortOffset)
    HebiArmPlugins.DoubledJointMirror(2, doubleShoulder)
};

end
