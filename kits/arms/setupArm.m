function [ group, kin, params ] = setupArm(kit, family)
% SETUPARM creates models for various pre-configured arm kits
%
%   [ group, kin, params ] = setupArm( kit, family )
%
% NOTE: If you are using different types of actuators or links, you will
% need to create a new custom script, or change this script accordingly.
%
%
% INPUTS:
% The 'kit' argument currently supports the following names:
%
%    '6-DoF + gripper', '6-DoF', '5-DoF', '4-DoF', '4-DoF SCARA', '3-DoF'
%
% The 'family' argument specifies the family name of the modules that
% should be selected. It is optional and defaults to 'Arm'.
%
% OUTPUTS:
% 'group' is the HebiGroup object for the modules in the arm that is used
% to control and get feedback from the arm
%
% 'kin' is the HebiKinematics object for calculating things like foward
% kinematics (FK), inverse kinematics (IK), Jacobians, gravity
% compensation, etc.
%
% 'params' is a struct that holds other information like the initialized
% directions of gravity, gripper information, torque offsets if using a gas
% spring to assist the robot, etc.  This struct can be a good place to put
% configuration-specific info as you start to customize the code.
%
% HEBI Robotics
% Jun 2018

if nargin < 2
   family = 'Arm'; 
end

shoulderJointComp = 0; % Nm  <--- Change this if you add a gas spring

%% Setup kinematic models
switch kit
    
    case '6-DoF + gripper'
        %%
        group = HebiLookup.newGroupFromNames(family, {
            'Base'
            'Shoulder'
            'Elbow'
            'Wrist1'
            'Wrist2'
            'Wrist3' });
        
        % Kinematic Model
        kin = HebiKinematics('hrdf/6-DoF_arm_w_gripper');
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains('gains/6-DoF_arm_gains');     
        group.send( 'gains', params.gains);
        
        % Settings / gains for the gripper spool to open-close the gripper
        params.gripperOpenEffort = 1;
        params.gripperCloseEffort = -5;
        params.gripperGains = [];
        
        % Compensation to joint efforts due to a gas spring (if present)
        params.effortOffset = [0 shoulderJointComp 0 0 0 0];

        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0.01 1.0 2.5 1.5 -1.5 0.01];
        
  
    case '6-DoF'
        %%
        group = HebiLookup.newGroupFromNames(family, {
            'Base'
            'Shoulder'
            'Elbow'
            'Wrist1'
            'Wrist2'
            'Wrist3' });
        
        % Kinematic Model
        kin = HebiKinematics('hrdf/6-DoF_arm');
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains('gains/6-DoF_arm_gains');     
        group.send( 'gains', params.gains);
        
        % Compensation to joint efforts due to a gas spring (if present)
        params.effortOffset = [0 shoulderJointComp 0 0 0 0];
        
        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0.01 1.0 2.5 1.5 -1.5 0.01];
        
        
    case '5-DoF' 
        %%
        group = HebiLookup.newGroupFromNames(family, {
            'Base'
            'Shoulder'
            'Elbow'
            'Wrist1'
            'Wrist2' });
        
        % Kinematic Model
        kin = HebiKinematics('hrdf/5-DoF_arm');
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains('gains/5-DoF_arm_gains');     
        group.send( 'gains', params.gains);
        
        % Account for external efforts due to the gas spring
        params.effortOffset = [0 shoulderJointComp 0 0 0];
        
        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0.01 1.0 2.5 1.5 -1.5];
        
        
    case '4-DoF'
        %%
        group = HebiLookup.newGroupFromNames(family, {
            'Base'
            'Shoulder'
            'Elbow'
            'Wrist1' });
        
        % Kinematic Model
        kin = HebiKinematics('hrdf/4-DoF_arm');
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains('gains/4-DoF_arm_gains');     
        group.send( 'gains', params.gains);
        
        % Account for external efforts due to the gas spring
        params.effortOffset = [0 shoulderJointComp 0 0 0];
        
        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0.01 1.0 2.5 1.5];
        
        
    case '4-DoF SCARA'
        %%
        group = HebiLookup.newGroupFromNames(family, {
            'Base'
            'Shoulder'
            'Elbow'
            'Wrist1' });
        
        % Kinematic Model
        kin = HebiKinematics('hrdf/4-DoF_arm_scara');
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains('gains/5-DoF_arm_scara_gains');     
        group.send( 'gains', params.gains);
        
        % Account for external efforts due to the gas spring
        params.effortOffset = [0 shoulderJointComp 0 0];
        
        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0.01 1.0 2.5 1.5];
        
        
    case '3-DoF'
        %%
        group = HebiLookup.newGroupFromNames(family, {
            'Base'
            'Shoulder'
            'Elbow' } );
        
        kin = HebiKinematics('/hrdf/3-DoF_arm');
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains('gains/3-DoF_arm_gains');     
        group.send( 'gains', params.gains);
        
        % Account for external efforts due to the gas spring
        params.effortOffset = [0 shoulderJointComp 0 0];
        
        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0.01 1.0 2.5];
        

    otherwise
        
        error([kit ' is not a supported kit name']);
        
end


%% Common Setup

% Determine initial gravity vector based on the internal pose filter of
% the base module.
fbk = group.getNextFeedbackFull();
baseRotMat = HebiUtils.quat2rotMat( [ fbk.orientationW(1), ...
                                      fbk.orientationX(1), ...
                                      fbk.orientationY(1), ...
                                      fbk.orientationZ(1) ] );
params.gravityVec = -baseRotMat(3,1:3);

end

