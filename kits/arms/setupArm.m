function [ group, kin, params ] = setupArm( kit, family, hasGasSpring )
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
% (These are the corresponding Assembly Part Numbers for the Arm Kits)
%
%   X-Series Arms
%    'A-2085-06G' (6-DoF X-Series Arm + Gripper), 
%    'A-2085-06' (6-DoF X-Series Arm),
%    'A-2085-05G' (5-DoF X-Series Arm + Gripper), 
%    'A-2085-05' (5-DoF X-Series Arm), 
%    'A-2085-04' (4-DoF X-Series Arm),
%    'A-2084-01' (4-DoF X-Series SCARA Arm),
%    'A-2085-03' (3-DoF X-Series Arm),
%   R-Series Arms
%    'A-2240-06G' (6-DoF R-Series Arm + Gripper), 
%    'A-2240-06' (6-DoF R-Series Arm),
%    'A-2240-05G' (5-DoF R-Series Arm + Gripper), 
%    'A-2240-05' (5-DoF R-Series Arm), 
%    'A-2240-04' (4-DoF R-Series Arm),
%    'A-2302-01' (4-DoF R-Series SCARA Arm)
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
    
    case 'A-2085-06G'
        %% X-Series 6-DoF Arm with Gripper
        group = HebiLookup.newGroupFromNames(family, {
            'J1_base'
            'J2_shoulder'
            'J3_elbow'
            'J4_wrist1'
            'J5_wrist2'
            'J6_wrist3' });
        
        % Kinematic Model
        kin = HebiKinematics([localDir '/hrdf/A-2085-06G']);
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains([localDir '/gains/A-2085-06']);     
        
        % Settings / gains for the gripper spool to open-close the gripper
        % X-Series Gripper Spool Assembly Part Number = A-2080-01
        params.hasGripper = true;
        params.gripperOpenEffort = 1;
        params.gripperCloseEffort = -5;
        params.gripperGains = HebiUtils.loadGains( ...
                                [localDir '/gains/A-2080-01'] );
        
        % Compensation to joint efforts due to a gas spring (if present)
        params.effortOffset = [0 shoulderJointComp 0 0 0 0];

        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0.01 1.0 2.5 1.5 -1.5 0.01];
        
    case 'A-2085-06'
        %% X-Series 6-DoF Arm
        group = HebiLookup.newGroupFromNames(family, {
            'J1_base'
            'J2_shoulder'
            'J3_elbow'
            'J4_wrist1'
            'J5_wrist2'
            'J6_wrist3' });
        
        % Kinematic Model
        kin = HebiKinematics([localDir '/hrdf/A-2085-06']);
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains([localDir '/gains/A-2085-06']);     
        
        % No Gripper
        params.hasGripper = false;
        
        % Compensation to joint efforts due to a gas spring (if present)
        params.effortOffset = [0 shoulderJointComp 0 0 0 0];
        
        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0.01 1.0 2.5 1.5 -1.5 0.01];
        
    case 'A-2085-05G'
        %% X-Series 5-DoF Arm with Gripper
        group = HebiLookup.newGroupFromNames(family, {
            'J1_base'
            'J2_shoulder'
            'J3_elbow'
            'J4_wrist1'
            'J5_wrist2' });
        
        % Kinematic Model
        kin = HebiKinematics([localDir '/hrdf/A-2085-05G']);
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains([localDir '/gains/A-2085-05']);     
        
        % Settings / gains for the gripper spool to open-close the gripper
        % X-Series Gripper Spool Assembly Part Number = A-2080-01
        params.hasGripper = true;
        params.gripperOpenEffort = 1;
        params.gripperCloseEffort = -5;
        params.gripperGains = HebiUtils.loadGains( ...
                                [localDir '/gains/A-2080-01'] );
        
        % Compensation to joint efforts due to a gas spring (if present)
        params.effortOffset = [0 shoulderJointComp 0 0 0];

        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0.01 1.0 2.5 1.5 -1.5];
               
    case 'A-2085-05' 
        %% X-Series 5-DoF Arm
        group = HebiLookup.newGroupFromNames(family, {
            'J1_base'
            'J2_shoulder'
            'J3_elbow'
            'J4_wrist1'
            'J5_wrist2' });
        
        % Kinematic Model
        kin = HebiKinematics([localDir '/hrdf/A-2085-05']);
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains([localDir '/gains/A-2085-05']);     
        
        % No Gripper
        params.hasGripper = false;
        
        % Account for external efforts due to the gas spring
        params.effortOffset = [0 shoulderJointComp 0 0 0];
        
        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0.01 1.0 2.5 1.5 -1.5];
        
    case 'A-2085-04'
        %% X-Series 4-DoF Arm
        group = HebiLookup.newGroupFromNames(family, {
            'J1_base'
            'J2_shoulder'
            'J3_elbow'
            'J4_wrist1' });
        
        % Kinematic Model
        kin = HebiKinematics([localDir '/hrdf/A-2085-04']);
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains([localDir '/gains/A-2085-04']);     
                
        % No Gripper
        params.hasGripper = false;
        
        % Account for external efforts due to the gas spring
        params.effortOffset = [0 shoulderJointComp 0 0];
        
        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0.01 1.0 2.5 1.5];
               
    case 'A-2084-01'
        %% X-Series 4-DoF SCARA Arm
        group = HebiLookup.newGroupFromNames(family, {
            'J1_base'
            'J2_shoulder'
            'J3_elbow'
            'J4_wrist1' });
        
        % Kinematic Model
        kin = HebiKinematics([localDir '/hrdf/A-2084-01']);
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains([localDir '/gains/A-2084-01']);     
                
        % No Gripper
        params.hasGripper = false;
        
        % Account for external efforts due to the gas spring
        params.effortOffset = [0 shoulderJointComp 0 0];
        
        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0.01 1.0 2.5 1.5];
                
    case 'A-2085-03'
        %% X-Series 3-DoF Arm
        group = HebiLookup.newGroupFromNames(family, {
            'J1_base'
            'J2_shoulder'
            'J3_elbow' });
        
        kin = HebiKinematics([localDir '/hrdf/A-2085-03']);
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains([localDir '/gains/A-2085-03']);     
                
        % No Gripper
        params.hasGripper = false;
        
        % Account for external efforts due to the gas spring
        params.effortOffset = [0 shoulderJointComp 0];
        
        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0.01 1.0 2.5];
        
    case 'A-2240-06G'
        %%  R-Series 6-DoF Arm with Gripper
        group = HebiLookup.newGroupFromNames(family, {
            'J1_base'
            'J2_shoulder'
            'J3_elbow'
            'J4_wrist1'
            'J5_wrist2'
            'J6_wrist3' });
        
        % Kinematic Model
        kin = HebiKinematics([localDir '/hrdf/A-2240-06G']);
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains([localDir '/gains/A-2240-06']);     
        
        % Settings / gains for the gripper spool to open-close the gripper
        % R-Series Gripper Spool Assembly Part Number = A-2255-01
        params.hasGripper = true;
        params.gripperOpenEffort = 1;
        params.gripperCloseEffort = -5;
        params.gripperGains = HebiUtils.loadGains( ...
                                [localDir '/gains/A-2255-01'] );
        
        % Compensation to joint efforts due to a gas spring (if present)
        params.effortOffset = [0 shoulderJointComp 0 0 0 0];

        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0.01 1.0 2.5 1.5 -1.5 0.01];
        
    case 'A-2240-06'
        %% R-Series 6-DoF Arm
        group = HebiLookup.newGroupFromNames(family, {
            'J1_base'
            'J2_shoulder'
            'J3_elbow'
            'J4_wrist1'
            'J5_wrist2'
            'J6_wrist3' });
        
        % Kinematic Model
        kin = HebiKinematics([localDir '/hrdf/A-2240-06']);
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains([localDir '/gains/A-2240-06']);     
        
        % No Gripper
        params.hasGripper = false;
        
        % Compensation to joint efforts due to a gas spring (if present)
        params.effortOffset = [0 shoulderJointComp 0 0 0 0];
        
        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0.01 1.0 2.5 1.5 -1.5 0.01];
        
    case 'A-2240-05G'
        %% R-Series 5-DoF Arm with Gripper
        group = HebiLookup.newGroupFromNames(family, {
            'J1_base'
            'J2_shoulder'
            'J3_elbow'
            'J4_wrist1'
            'J5_wrist2' });
        
        % Kinematic Model
        kin = HebiKinematics([localDir '/hrdf/A-2240-05G']);
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains([localDir '/gains/A-2240-05']);     
        
        % Settings / gains for the gripper spool to open-close the gripper
        % R-Series Gripper Spool Assembly Part Number = A-2255-01
        params.hasGripper = true;
        params.gripperOpenEffort = 1;
        params.gripperCloseEffort = -5;
        params.gripperGains = HebiUtils.loadGains( ...
                                [localDir '/gains/A-2255-01'] );
        
        % Compensation to joint efforts due to a gas spring (if present)
        params.effortOffset = [0 shoulderJointComp 0 0 0];

        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0.01 1.0 2.5 1.5 -1.5];
           
    case 'A-2240-05' 
        %% R-Series 5-DoF Arm
        group = HebiLookup.newGroupFromNames(family, {
            'J1_base'
            'J2_shoulder'
            'J3_elbow'
            'J4_wrist1'
            'J5_wrist2' });
        
        % Kinematic Model
        kin = HebiKinematics([localDir '/hrdf/A-2085-05']);
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains([localDir '/gains/A-2240-05']);     
        
        % No Gripper
        params.hasGripper = false;
        
        % Account for external efforts due to the gas spring
        params.effortOffset = [0 shoulderJointComp 0 0 0];
        
        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0.01 1.0 2.5 1.5 -1.5];
        
    case 'A-2240-04'
        %% R-Series 4-DoF Arm
        group = HebiLookup.newGroupFromNames(family, {
            'J1_base'
            'J2_shoulder'
            'J3_elbow'
            'J4_wrist1' });
        
        % Kinematic Model
        kin = HebiKinematics([localDir '/hrdf/A-2240-04']);
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains([localDir '/gains/A-2240-04']);     
                
        % No Gripper
        params.hasGripper = false;
        
        % Account for external efforts due to the gas spring
        params.effortOffset = [0 shoulderJointComp 0 0];
        
        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0.01 1.0 2.5 1.5];    
        
    case 'A-2302-01'
        %% R-Series 4-DoF SCARA Arm
        group = HebiLookup.newGroupFromNames(family, {
            'J1_base'
            'J2_shoulder'
            'J3_elbow'
            'J4_wrist1' });
        
        % Kinematic Model
        kin = HebiKinematics([localDir '/hrdf/A-2302-01']);
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains([localDir '/gains/A-2302-01']);
        
        % No Gripper
        params.hasGripper = false;
        
        % Account for external efforts due to the gas spring
        params.effortOffset = [0 shoulderJointComp 0 0];
        
        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0.01 1.0 2.5 1.5];
        
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

