function [ arm, params, gripper ] = setupArm( kit, family, hasGasSpring )
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
%    'A-2303-01' (7-DoF R-Series Double Shoulder Arm)
%    'A-2303-01G' (7-DoF R-Series Double Shoulder Arm + Gripper)
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

if nargin < 3 || isempty(hasGasSpring)
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
        kin = HebiUtils.loadHRDF([localDir '/hrdf/A-2085-06G']);
        
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
        kin = HebiUtils.loadHRDF([localDir '/hrdf/A-2085-06']);
        
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
        kin = HebiUtils.loadHRDF([localDir '/hrdf/A-2085-05G']);
        
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
        kin = HebiUtils.loadHRDF([localDir '/hrdf/A-2085-05']);
        
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
            'J4_wrist' });
        
        % Kinematic Model
        kin = HebiUtils.loadHRDF([localDir '/hrdf/A-2085-04']);
        
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
            'J4_wrist' });
        
        % Kinematic Model
        kin = HebiUtils.loadHRDF([localDir '/hrdf/A-2084-01']);
        
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
        
        kin = HebiUtils.loadHRDF([localDir '/hrdf/A-2085-03']);
        
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
        kin = HebiUtils.loadHRDF([localDir '/hrdf/A-2240-06G']);
        
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
        kin = HebiUtils.loadHRDF([localDir '/hrdf/A-2240-06']);
        
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
        kin = HebiUtils.loadHRDF([localDir '/hrdf/A-2240-05G']);
        
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
        kin = HebiUtils.loadHRDF([localDir '/hrdf/A-2085-05']);
        
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
            'J4_wrist' });
        
        % Kinematic Model
        kin = HebiUtils.loadHRDF([localDir '/hrdf/A-2240-04']);
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains([localDir '/gains/A-2240-04']);     
                
        % No Gripper
        params.hasGripper = false;
        
        % Account for external efforts due to the gas spring
        params.effortOffset = [0 shoulderJointComp 0 0];
        
        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0.01 1.0 2.5 1.5];    
        
    case 'Maggie7dof'
        %% R-Series 6-DoF Arm
        group = HebiLookup.newGroupFromNames(family, {
            'J1_base'
            'J2_shoulder1'
            'J3_shoulder2'
            'J4_elbow1'
            'J5_elbow2'
            'J6_wrist1'
            'J7_wrist2'});
        
        % Kinematic Model
        kin = HebiUtils.loadHRDF([localDir '/hrdf/7-DoF-Maggie']);
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains([localDir '/gains/A-2240-06']);     
        
        % No Gripper
        params.hasGripper = false;
        
        
        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0.01 1.0 2.5 1.5 -1.5 0.01];
        
    case 'A-2302-01'
        %% R-Series 4-DoF SCARA Arm
        group = HebiLookup.newGroupFromNames(family, {
            'J1_base'
            'J2_shoulder'
            'J3_elbow'
            'J4_wrist' });
        
        % Kinematic Model
        kin = HebiUtils.loadHRDF([localDir '/hrdf/A-2302-01']);
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains([localDir '/gains/A-2302-01']);
        
        % No Gripper
        params.hasGripper = false;
        
        % Account for external efforts due to the gas spring
        params.effortOffset = [0 shoulderJointComp 0 0];
        
        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0.01 1.0 2.5 1.5];
            
        
    case 'A-2099-07'
        %% X-Series 7-DoF Double Shoulder Arm with Gripper
        group = HebiLookup.newGroupFromNames(family, {
            'J1_base'
            'J2A_shoulder1'
            'J3_shoulder2'
            'J4_elbow'
            'J5_wrist1'
            'J6_wrist2'
            'J7_wrist3' });
        
        % Kinematic Model
        kin = HebiKinematics([localDir '/hrdf/A-2099-07']);
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains([localDir '/gains/A-2099-07']);
        % Get gains from shoulder to send to doubleShoulder
        mainShoulder = HebiLookup.newGroupFromNames(family, 'J2A_shoulder1');
        
        % Has Gripper
        params.hasGripper = false;
        
        % Account for external efforts due to the gas spring
        params.effortOffset = [0 shoulderJointComp 0 0 0 0 0];
        
        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0 -1.25 0 2 -3.5 -1.25 0];
        
        % Default plugins
        doubleShoulder = HebiLookup.newGroupFromNames(family, 'J2B_shoulder1');
        
    case 'A-2099-07G'
        %% X-Series 7-DoF Double Shoulder Arm with Gripper
        group = HebiLookup.newGroupFromNames(family, {
            'J1_base'
            'J2A_shoulder1'
            'J3_shoulder2'
            'J4_elbow'
            'J5_wrist1'
            'J6_wrist2'
            'J7_wrist3' });
        
        % Kinematic Model
        kin = HebiKinematics([localDir '/hrdf/A-2099-07G']);
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains([localDir '/gains/A-2099-07G']);
        % Get gains from shoulder to send to doubleShoulder
        mainShoulder = HebiLookup.newGroupFromNames(family, 'J2A_shoulder1');
        
        % Has Gripper
        % X-Series Gripper Spool Assembly Part Number = A-2080-01
        params.hasGripper = true;
        params.gripperOpenEffort = 1;
        params.gripperCloseEffort = -5;
        params.gripperGains = HebiUtils.loadGains( ...
            [localDir '/gains/A-2080-01'] );
        
        % Account for external efforts due to the gas spring
        params.effortOffset = [0 shoulderJointComp 0 0 0 0 0];
        
        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0 -1.25 0 2 -3.5 -1.25 0];
        
        % Default plugins
        doubleShoulder = HebiLookup.newGroupFromNames(family, 'J2B_shoulder1');
        
    case 'A-2303-01'
        %% R-Series 7-DoF Double Shoulder Arm
        group = HebiLookup.newGroupFromNames(family, {
            'J1_base'
            'J2A_shoulder1'
            'J3_shoulder2'
            'J4_elbow1'
            'J5_elbow2'
            'J6_wrist1'
            'J7_wrist2' });
        
        % Kinematic Model
        kin = HebiKinematics([localDir '/hrdf/A-2303-01']);
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains([localDir '/gains/A-2303-01']);
        % Get gains from shoulder to send to doubleShoulder
        mainShoulder = HebiLookup.newGroupFromNames(family, 'J2A_shoulder1');
        
        % Has Gripper
        params.hasGripper = false;
        
        % Account for external efforts due to the gas spring
        params.effortOffset = [0 shoulderJointComp 0 0 0 0 0];
        
        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0 -1.25 0 2 0 1.25 0];
        
        % Default plugins
        doubleShoulder = HebiLookup.newGroupFromNames(family, 'J2B_shoulder1');  
        
    case 'Maggie7Dof'
        %% R-Series 7-DoF single shoulder arm on Maggie
        group = HebiLookup.newGroupFromNames(family, {
            'J1_base'
            'J2_shoulder1'
            'J3_shoulder2'
            'J4_elbow1'
            'J5_elbow2'
            'J6_wrist1'
            'J7_wrist2' });
        
        % Kinematic Model
        kin = HebiKinematics([localDir '/hrdf/7-DoF-Maggie']);
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains([localDir '/gains/A-2303-01']);
        
        % Has Gripper
        params.hasGripper = false;
        
        % Account for external efforts due to the gas spring
        params.effortOffset = [0 shoulderJointComp 0 0 0 0 0];
        
        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0 -1.25 0 2 0 1.25 0];
        
        
    case 'A-2303-01G'
        %% R-Series 7-DoF Double Shoulder Arm with Gripper
        group = HebiLookup.newGroupFromNames(family, {
            'J1_base'
            'J2A_shoulder1'
            'J3_shoulder2'
            'J4_elbow1'
            'J5_elbow2'
            'J6_wrist1'
            'J7_wrist2' });
        
        % Kinematic Model
        kin = HebiKinematics([localDir '/hrdf/A-2303-01G']);
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains([localDir '/gains/A-2303-01']);
        % Get gains from shoulder to send to doubleShoulder
        mainShoulder = HebiLookup.newGroupFromNames(family, 'J2A_shoulder1');
        
        % Has Gripper
        % R-Series Gripper Spool Assembly Part Number = A-2255-01
        params.hasGripper = true;
        params.gripperOpenEffort = 1;
        params.gripperCloseEffort = -5;
        params.gripperGains = HebiUtils.loadGains( ...
            [localDir '/gains/A-2255-01'] );
        
        % Account for external efforts due to the gas spring
        params.effortOffset = [0 shoulderJointComp 0 0 0 0 0];
        
        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0 -1.25 0 2 0 1.25 0];
        
        % Default plugins
        doubleShoulder = HebiLookup.newGroupFromNames(family, 'J2B_shoulder1');

    case 'A-2303-06'
        %% R-Series 6-DoF Double Shoulder Arm
        group = HebiLookup.newGroupFromNames(family, {
            'J1_base'
            'J2A_shoulder'
            'J3_elbow1'
            'J4_elbow2'
            'J5_wrist1'
            'J6_wrist2' });
        
        % Kinematic Model
        kin = HebiKinematics([localDir '/hrdf/A-2303-06']);
        
        % Load and send arm gains
        params.gains = HebiUtils.loadGains([localDir '/gains/A-2303-06']);
        % Get gains from shoulder to send to doubleShoulder
        mainShoulder = HebiLookup.newGroupFromNames(family, 'J2A_shoulder');
        
        % Has Gripper
        params.hasGripper = false;
        
        % Account for external efforts due to the gas spring
        params.effortOffset = [0 shoulderJointComp 0 0 0 0];
        
        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0 -1.25 0.5 0 1 0];
        
        % Default plugins
        doubleShoulder = HebiLookup.newGroupFromNames(family, 'J2B_shoulder');
        
    otherwise
        
%     otherwise
%         
%         error([kit ' is not a supported kit name']);
        
end


%% Common Setup
arm = HebiArm(group, kin);
HebiUtils.sendWithRetry(arm.group, 'gains', params.gains);

if contains(kit,'A-2099-07') || contains(kit,'A-2303')
    arm.plugins = {
        HebiArmPlugins.EffortOffset(params.effortOffset)
        HebiArmPlugins.DoubledJointMirror(2, doubleShoulder)
        };
    shoulderGains = mainShoulder.getGains();
    HebiUtils.sendWithRetry(doubleShoulder, 'gains', shoulderGains);
else
    arm.plugins = {
        HebiArmPlugins.EffortOffset(params.effortOffset)
        };
end

% Setup gripper
gripper = [];
if params.hasGripper
    
    gripperGroup = HebiLookup.newGroupFromNames( family, 'gripperSpool' );
    HebiUtils.sendWithRetry(gripperGroup, 'gains', params.gripperGains);
    
    gripper = HebiGripper(gripperGroup);
    gripper.openEffort = params.gripperOpenEffort;
    gripper.closeEffort = params.gripperCloseEffort;
    
end

end

