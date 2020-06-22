function [ arm, params, gripper ] = setupArmWithGripper(family)

% Arm Module Names
group = HebiLookup.newGroupFromNames(family, {
    'J1_base'
    'J2A_shoulder'
    'J3_elbow'
    'J4_wrist1'
    'J5_wrist2'
    'J6_wrist3' });

% Load and send Gains
params.gains = HebiUtils.loadGains('gains/6-dof-arm-gains-rosie');
HebiUtils.sendWithRetry(group, 'gains', params.gains);

% Kinematic Model
kin = HebiKinematics('hrdf/6-DoF_arm_w_gripper'); % TODO: change hrdf to double shoulder

% Gripper
gripperGroup = HebiLookup.newGroupFromNames(family, 'Spool');
params.gripperGains = HebiUtils.loadGains('gains/gripper-gains');
HebiUtils.sendWithRetry(gripperGroup, 'gains', params.gripperGains);

% API Wrappers
arm = HebiArm(group, kin);
gripper = HebiGripper(gripperGroup);

% Default seed positions for doing inverse kinematics
params.ikSeedPos = [0 1 2.5 1.5 -1.5 1];

% Trajectory generator parameters
params.minTrajDuration = 0.33; % [sec]
params.defaultSpeedFactor = 0.9;
arm.trajGen.setMinDuration(params.minTrajDuration);
arm.trajGen.setSpeedFactor(params.defaultSpeedFactor);

% (Optional) Compensation to joint efforts due to a gas spring (if present)
shoulderJointComp = 0; % Nm  <--- Change this if you add a gas spring
params.effortOffset = [0 shoulderJointComp 0 0 0 0];

% Default plugins
doubleShoulder = HebiLookup.newGroupFromNames(family, 'J2B_shoulder');
arm.plugins = {
    HebiArmPlugins.EffortOffset(params.effortOffset)
    HebiArmPlugins.DoubledJointMirror(2, doubleShoulder)
};

end
