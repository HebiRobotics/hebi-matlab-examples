function [ arm, userData, gripper ] = setupArm( kit )
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
%    'A-2099-01G' (7-DoF Double-shouldered X-Series Arm + Gripper), 
%    'A-2099-01' (7-DoF Double-shouldered X-Series Arm),
%    'A-2085-06G' (6-DoF X-Series Arm + Gripper), 
%    'A-2085-06' (6-DoF X-Series Arm),
%    'A-2085-05G' (5-DoF X-Series Arm + Gripper), 
%    'A-2085-05' (5-DoF X-Series Arm), 
%    'A-2085-04' (4-DoF X-Series Arm),
%    'A-2084-01' (4-DoF X-Series SCARA Arm),
%    'A-2085-03' (3-DoF X-Series Arm),
%   R-Series Arms
%    'A-2303-01G' (7-DoF Double-shouldered R-Series Arm + Gripper), 
%    'A-2303-01' (7-DoF Double-shouldered R-Series Arm),
%    'A-2240-06G' (6-DoF R-Series Arm + Gripper), 
%    'A-2240-06' (6-DoF R-Series Arm),
%    'A-2240-05G' (5-DoF R-Series Arm + Gripper), 
%    'A-2240-05' (5-DoF R-Series Arm), 
%    'A-2240-04' (4-DoF R-Series Arm),
%    'A-2302-01' (4-DoF R-Series SCARA Arm)
%   T-Series Arms
%    'A-2582-01G' (7-DoF Double-shouldered T-Series Arm + Gripper), 
%    'A-2582-01' (7-DoF Double-shouldered T-Series Arm),
%    'A-2580-06G' (6-DoF T-Series Arm + Gripper), 
%    'A-2580-06' (6-DoF T-Series Arm),
%    'A-2580-05G' (5-DoF T-Series Arm + Gripper), 
%    'A-2580-05' (5-DoF T-Series Arm), 
%    'A-2580-04' (4-DoF T-Series Arm),
%    'A-2590-01' (4-DoF T-Series SCARA Arm)
%
% The 'family' argument specifies the family name of the modules that
% should be selected.
%
% The 'hasGasSpring' argument specifies whether there is a gas spring
% supporting the shoulder joint of the arm to provide extra payload.  If it
% is not specified it defaults to 'false'.
%
% OUTPUTS:
% 'arm' is the HebiArm object that contains the group and kinematic model
% for a given arm.
%
% 'userData' is a struct that holds other information like the initialized
% directions of gravity, gripper information, torque offsets if using a gas
% spring to assist the robot, etc.  This struct can be a good place to put
% configuration-specific info as you start to customize the code.
%
% HEBI Robotics
% Jun 2018

%% Load config file
localDir = fileparts(mfilename('fullpath'));
configFile = fullfile(localDir, 'config', [kit '.cfg.yaml']);
config = HebiUtils.loadRobotConfig(configFile);

% Initialize params with userData. May contain settings for ik seeds and
% efforts to open-close the gripper
userData = config.userData;
userData.localDir = localDir;

%% Create comms and models according to config
arm = HebiArm.createFromConfig(config);

%% Create optional gripper based on convention

% Common fields
gripper = [];
userData.has_gripper = isfield(userData, 'has_gripper');

% Setup
if userData.has_gripper

    % Load and set gains
    userData.gripperGains = HebiUtils.loadGains(config.gains.gripper);
    gripperGroup = HebiLookup.newGroupFromNames( config.families(1), 'gripperSpool' );
    HebiUtils.sendWithRetry(gripperGroup, 'gains', userData.gripperGains);
    
    % Other params
    gripper = HebiGripper(gripperGroup);
    gripper.openEffort = userData.gripper_open_effort;
    gripper.closeEffort = userData.gripper_close_effort;
    
end

end

