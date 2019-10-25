function [ group, kin, params ] = setupArm( kit, family, series, hasGasSpring )
% SETUPARM creates models and loads parameters for controlling various 
% pre-configured arm kits.
%
%   [ group, kin, params ] = setupArm( kit, family )
%
% OVERVIEW:
%   - Creates a group to communicate to the actuators in an arm
%   - Loads gains for the arm from an XML file and sends them
%   - Loads the kinematic description from the arm from an HRDF file
%   - Sets initial seed positions for doing inverse kinematics (IK)
%   - Sets effort offsets if a gas spring is used to assist the shoulder
%   - Sets up a group and gains for a gripper, specified
%
% If you are using different types of actuators or links, you will
% need to create a new HRDF file with the appropriate kinematics, an XML
% file with the appropriate gains, and modify this script to load those new
% files and set any other application-specific parameters you may have.
%
% INPUTS:
% The 'kit' argument currently supports the following names:
%
%    '6-DoF + gripper', 
%    '6-DoF'
%    '5-DoF + gripper', 
%    '5-DoF'
%    '4-DoF
%    '4-DoF SCARA'
%    '3-DoF'
%
% The 'family' argument specifies the family name of the modules that
% should be selected.
%
% The 'hasGasSpring' argument specifies whether there is a gas spring
% supporting the shoulder joint of the arm to provide extra payload.  If it
% is not specified it defaults to 'false'.
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

localDir = fileparts(mfilename('fullpath'));
params.localDir = localDir;

series = lower( series );

if nargin < 4 || isempty(hasGasSpring)
   hasGasSpring = false;
end

if hasGasSpring
    shoulderJointComp = -7; % Nm  <--- This should be around -7 Nm for most
                            %          kits, but it may need to be tuned
                            %          for your specific setup.
else
    shoulderJointComp = 0;
end


%% Setup kinematic models
switch kit
    
    case '6-DoF + gripper'
        %%
        group = HebiLookup.newGroupFromNames(family, {
            'base'
            'shoulder'
            'elbow'
            'wrist1'
            'wrist2'
            'wrist3' });
        
        % Kinematic Model
        kin = HebiKinematics([localDir '/hrdf/' series '/6-DoF_arm_w_gripper']);
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains([localDir '/gains/' series '/6-DoF_arm_gains']);     
        
        % Settings / gains for the gripper spool to open-close the gripper
        params.hasGripper = true;
        params.gripperOpenEffort = 1;
        params.gripperCloseEffort = -5;
        params.gripperGains = HebiUtils.loadGains( ...
                                [localDir '/gains/gripper_spool_gains'] );
        
        % Compensation to joint efforts due to a gas spring (if present)
        params.effortOffset = [0 shoulderJointComp 0 0 0 0];

        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0.01 1.0 2.5 1.5 -1.5 0.01];
        
  
    case '6-DoF'
        %%
        group = HebiLookup.newGroupFromNames(family, {
            'base'
            'shoulder'
            'elbow'
            'wrist1'
            'wrist2'
            'wrist3' });
        
        % Kinematic Model
        kin = HebiKinematics([localDir '/hrdf/' series '/6-DoF_arm']);
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains([localDir '/gains/' series '/6-DoF_arm_gains']);     
        
        % No Gripper
        params.hasGripper = false;
        
        % Compensation to joint efforts due to a gas spring (if present)
        params.effortOffset = [0 shoulderJointComp 0 0 0 0];
        
        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0.01 1.0 2.5 1.5 -1.5 0.01];
        
    case '5-DoF + gripper'
        %%
        group = HebiLookup.newGroupFromNames(family, {
            'base'
            'shoulder'
            'elbow'
            'wrist1'
            'wrist2' });
        
        % Kinematic Model
        kin = HebiKinematics([localDir '/hrdf/' series '/5-DoF_arm_w_gripper']);
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains([localDir '/gains/' series '/5-DoF_arm_gains']);     
        
        % Settings / gains for the gripper spool to open-close the gripper
        params.hasGripper = true;
        params.gripperOpenEffort = 1;
        params.gripperCloseEffort = -5;
        params.gripperGains = HebiUtils.loadGains( ...
                                [localDir '/gains/gripper_spool_gains'] );
        
        % Compensation to joint efforts due to a gas spring (if present)
        params.effortOffset = [0 shoulderJointComp 0 0 0];

        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0.01 1.0 2.5 1.5 -1.5];
        
        
    case '5-DoF' 
        %%
        group = HebiLookup.newGroupFromNames(family, {
            'base'
            'shoulder'
            'elbow'
            'wrist1'
            'wrist2' });
        
        % Kinematic Model
        kin = HebiKinematics([localDir '/hrdf/' series '/5-DoF_arm']);
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains([localDir '/gains/' series '/5-DoF_arm_gains']);     
        
        % No Gripper
        params.hasGripper = false;
        
        % Account for external efforts due to the gas spring
        params.effortOffset = [0 shoulderJointComp 0 0 0];
        
        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0.01 1.0 2.5 1.5 -1.5];
        
        
    case '4-DoF'
        %%
        group = HebiLookup.newGroupFromNames(family, {
            'base'
            'shoulder'
            'elbow'
            'wrist1' });
        
        % Kinematic Model
        kin = HebiKinematics([localDir '/hrdf/' series '/4-DoF_arm']);
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains([localDir '/gains/' series '/4-DoF_arm_gains']);     
                
        % No Gripper
        params.hasGripper = false;
        
        % Account for external efforts due to the gas spring
        params.effortOffset = [0 shoulderJointComp 0 0];
        
        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0.01 1.0 2.5 1.5];
        
        
    case '4-DoF SCARA'
        %%
        group = HebiLookup.newGroupFromNames(family, {
            'base'
            'shoulder'
            'elbow'
            'wrist1' });
        
        % Kinematic Model
        kin = HebiKinematics([localDir '/hrdf/' series '/4-DoF_arm_scara']);
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains([localDir '/gains/' series '/5-DoF_arm_scara_gains']);     
                
        % No Gripper
        params.hasGripper = false;
        
        % Account for external efforts due to the gas spring
        params.effortOffset = [0 shoulderJointComp 0 0];
        
        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0.01 1.0 2.5 1.5];
        
        
    case '3-DoF'
        %%
        group = HebiLookup.newGroupFromNames(family, {
            'base'
            'shoulder'
            'elbow' } );
        
        kin = HebiKinematics([localDir '/hrdf/' series '/3-DoF_arm']);
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains([localDir '/gains/' series '/3-DoF_arm_gains']);     
                
        % No Gripper
        params.hasGripper = false;
        
        % Account for external efforts due to the gas spring
        params.effortOffset = [0 shoulderJointComp 0];
        
        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0.01 1.0 2.5];
        

    otherwise
        
        error([kit ' is not a supported kit name']);
        
end


%% Common Setup

% Set the gains on the arm, set a bunch of times in a loop to make
% absolutely sure they get set.
numSends = 20;
for i=1:numSends
    fbk = group.getNextFeedback();
    group.send('gains',params.gains);
end

% Determine initial gravity vector based on the internal pose filter of
% the base module.
fbk = group.getNextFeedbackFull();
baseRotMat = HebiUtils.quat2rotMat( [ fbk.orientationW(1), ...
                                      fbk.orientationX(1), ...
                                      fbk.orientationY(1), ...
                                      fbk.orientationZ(1) ] );
params.gravityVec = -baseRotMat(3,1:3);

end

