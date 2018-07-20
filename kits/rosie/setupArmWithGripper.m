function [ params, armKin, trajGen ] = setupArmWithGripper( )

    % Kinematic Model
    armKin = HebiKinematics('hrdf/6-DoF_arm_w_gripper');
    
    % Arm Gains
    params.armGains = HebiUtils.loadGains('gains/6-DoF-Arm-Gains-Rosie');
    params.gripperGains = [];

    % Compensation to joint efforts due to a gas spring (if present)
    shoulderJointComp = 0; % Nm  <--- Change this if you add a gas spring
    params.effortOffset = [0 shoulderJointComp 0 0 0 0];
    
    % Torques for the gripper spool to open-close the gripper
    params.gripperOpenEffort = 1;
    params.gripperCloseEffort = -5;
    
    % Default seed positions for doing inverse kinematics
    params.ikSeedPos = [0 1 2.5 1.5 -1.5 1];
    
    % Trajectory generator
    trajGen = HebiTrajectoryGenerator(armKin);
    trajGen.setMinDuration(0.2); % [sec]
    trajGen.setSpeedFactor(0.9);
end
