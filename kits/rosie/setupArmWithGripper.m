function [ params, armKin, trajGen ] = setupArmWithGripper( )

    % Kinematic Model
    armKin = HebiKinematics('hrdf/6-DoF_arm_w_gripper');
    
    % Arm Module Names and Gains
    params.armModuleNames = { 'Base', 'Shoulder', 'Elbow', ...
                              'Wrist1', 'Wrist2', 'Wrist3' };   
    params.armGains = HebiUtils.loadGains('gains/6-DoF-Arm-Gains-Rosie');
    
    % Gripper Module Name and Gains
    params.gripperModuleNames = { 'Spool' };
    params.gripperGains = [];

    % Compensation to joint efforts due to a gas spring (if present)
    shoulderJointComp = 0; % Nm  <--- Change this if you add a gas spring
    params.effortOffset = [0 shoulderJointComp 0 0 0 0];

    
    params.gripperGains = [];
    
    % Torques for the gripper spool to open-close the gripper
    params.gripperOpenEffort = 1;
    params.gripperCloseEffort = -5;
    
    % Default seed positions for doing inverse kinematics
    params.ikSeedPos = [0 1 2.5 1.5 -1.5 1];
    
    % Trajectory generator
    params.minTrajDuration = 0.33; % [sec]
    params.defaultSpeedFactor = 0.9;
    
    trajGen = HebiTrajectoryGenerator(armKin);
    trajGen.setMinDuration( params.minTrajDuration ); 
    trajGen.setSpeedFactor( params.defaultSpeedFactor );
end
