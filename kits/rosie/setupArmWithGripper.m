function [ params, armKin, trajGen ] = setupArmWithGripper( )

    % Approximate kinematics to the tip of the gripper, expressed in the
    % output frame of the last module on the arm
    gripperOutput = eye(4);
    gripperOutput(1:3,4) = [0; 0; .075];

    % Kinematic Model
    % 6-DoF Arm w/ Gripper
    armKin = HebiKinematics();
    armKin.addBody('X8-9');
    armKin.addBody('X5-HeavyBracket', 'mount', 'right-inside');
    armKin.addBody('X8-16'); 
    armKin.addBody('X5-Link', 'extension', 0.325, 'twist', pi, ...
                    'mass', .500); % Added mass for the gripper spool
    armKin.addBody('X8-9');
    armKin.addBody('X5-Link', 'extension', 0.325, 'twist', pi);
    armKin.addBody('X5-4');
    armKin.addBody('X5-LightBracket', 'mount', 'right');
    armKin.addBody('X5-1');
    armKin.addBody('X5-LightBracket', 'mount', 'right');
    armKin.addBody('X5-1');
    armKin.addBody( 'GenericLink', 'CoM', [0 0 .025], ...
                                'Output', gripperOutput, ...
                                'Mass', .300 );

    % Compensation to joint efforts due to a gas spring (if present)
    shoulderJointComp = 0; % Nm  <--- Change this if you add a gas spring
    params.effortOffset = [0 shoulderJointComp 0 0 0 0];
    
    % Torques for the gripper spool to open-close the gripper
    params.gripperOpenEffort = 1;
    params.gripperCloseEffort = -5;
    
    % Default seed positions for doing inverse kinematics
    params.ikSeedPos = [0 1 2.5 1.5 -1.5 1];
    
    % Arm Gains
    params.armGains = HebiUtils.loadGains('6-DoF-Arm-Gains-Rosie');
    params.gripperGains = [];


    % Trajectory generator
    trajGen = HebiTrajectoryGenerator(armKin);
    trajGen.setMinDuration(0.2); % Speed up 'small' movements (default is >1s)
    trajGen.setSpeedFactor(0.9);
end
