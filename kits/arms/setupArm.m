function [ group, kin, params ] = setupArm(kit, family)
% SETUPARM creates models for various pre-configured arm kits
%
%   [ group, kin, params ] = setupArm(kitName, family)
%
% NOTE: If you are using different types of actuators or links, you will
% need to create a new custom script, or change this script accordingly.
%
%
% INPUTS:
% The 'kitName' argument currently supports the following names:
%
%    '6dof_w_gripper', '6dof', '5dof', '4dof', '4dof-scara'
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

%% Setup kinematic models
switch kit
    
    case '6dof_w_gripper' % A-2084-06
        %%
        % Create group for communications
        group = HebiLookup.newGroupFromNames(family, {
            'Base'
            'Shoulder'
            'Elbow'
            'Wrist1'
            'Wrist2'
            'Wrist3' });
        
        % Approximate kinematics to the tip of the gripper, expressed in the
        % output frame of the last module on the arm
        gripperOutput = eye(4);
        gripperOutput(1:3,4) = [0; 0; .075];

        % Kinematic Model
        % 6-DoF Arm w/ Gripper
        kin = HebiKinematics();
        kin.addBody('X8-9');
        kin.addBody('X5-HeavyBracket', 'mount', 'right-inside');
        kin.addBody('X8-16'); 
        kin.addBody('X5-Link', 'extension', 0.325, 'twist', pi, ...
                        'mass', .500); % Added mass for the gripper spool
        kin.addBody('X8-9');
        kin.addBody('X5-Link', 'extension', 0.325, 'twist', pi);
        kin.addBody('X5-4');
        kin.addBody('X5-LightBracket', 'mount', 'right');
        kin.addBody('X5-1');
        kin.addBody('X5-LightBracket', 'mount', 'right');
        kin.addBody('X5-1');
        kin.addBody( 'GenericLink', 'CoM', [0 0 .025], ...
                                    'Output', gripperOutput, ...
                                    'Mass', .100 );

        % Compensation to joint efforts due to a gas spring (if present)
        shoulderJointComp = 0; % Nm  <--- Change this if you add a gas spring
        params.effortOffset = [0 shoulderJointComp 0 0 0 0];

        % Torques for the girpper spool to open-close the gripper
        params.gripperOpenEffort = 1;
        params.gripperCloseEffort = -5;

        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0 1 2.5 1.5 -1.5 1];
        
        % Load and send the gains
        gains = HebiUtils.loadGains('6-DoF-Arm-Gains');
        
        gains.positionKp = gains.positionKp / 2;
        
        group.send('gains',gains);
        params.gains = gains;
  
    case '6dof' % A-2084-06
        %%
        % Create group for communications
        group = HebiLookup.newGroupFromNames(family, {
            'Base'
            'Shoulder'
            'Elbow'
            'Wrist1'
            'Wrist2'
            'Wrist3' });
        
        % Kinematic Model
        kin = HebiKinematics();
        kin.addBody('X8-9');
        kin.addBody('X5-HeavyBracket', 'mount', 'right-outside');
        kin.addBody('X8-9', 'PosLim', [-0.6 1.2]); % gas spring limits
        kin.addBody('X5-Link', 'extension', 0.325, 'twist', pi);
        kin.addBody('X5-9');
        kin.addBody('X5-Link', 'extension', 0.325, 'twist', pi);
        kin.addBody('X5-1');
        kin.addBody('X5-LightBracket', 'mount', 'right');
        kin.addBody('X5-1');
        kin.addBody('X5-LightBracket', 'mount', 'right');
        kin.addBody('X5-1');
        
        % Compensation to joint efforts due to a gas spring (if present)
        shoulderJointComp = 0; % Nm  <--- Change this if you add a gas spring
        params.effortOffset = [0 shoulderJointComp 0 0 0 0];
        
        % Default seed positions for doing inverse kinematics
        params.ikSeedPos = [0 1 2.5 1.5 -1.5 1];
        
        % Load and send the gains
        gains = HebiUtils.loadGains('6-DoF-Arm-Gains');
        
        gains.positionKp = gains.positionKp / 2;
        
        group.send('gains',gains);
        params.gains = gains;
        
    case '5dof' % A-2084-05
        %%
        % Create group for communications
        group = HebiLookup.newGroupFromNames(family, {
            'Base'
            'Shoulder'
            'Elbow'
            'Wrist1'
            'Wrist2' });
        
        % Kinematic Model
        kin = HebiKinematics();
        kin.addBody('X8-9');
        kin.addBody('X5-HeavyBracket', 'mount', 'right-outside');
        kin.addBody('X8-9', 'PosLim', [-0.6 1.2]); % gas spring limits
        kin.addBody('X5-Link', 'extension', 0.325, 'twist', pi);
        kin.addBody('X5-9');
        kin.addBody('X5-Link', 'extension', 0.325, 'twist', pi);
        kin.addBody('X5-1');
        kin.addBody('X5-LightBracket', 'mount', 'right');
        kin.addBody('X5-1');
        
        % Account for external efforts due to the gas spring
        params.effortOffset = [0 -9.8 0 0 0];
        
    case '4dof' % A-2085-04
        %%
        % Create group for communications
        group = HebiLookup.newGroupFromNames(family, {
            'Base'
            'Shoulder'
            'Elbow'
            'Wrist1' });
        
        % Kinematic Model
        kin = HebiKinematics();
        kin.addBody('X5-4');
        kin.addBody('X5-HeavyBracket', 'mount', 'right-outside');
        kin.addBody('X5-9');
        kin.addBody('X5-Link', 'extension', 0.325, 'twist', pi);
        kin.addBody('X5-4');
        kin.addBody('X5-Link', 'extension', 0.325, 'twist', pi);
        kin.addBody('X5-1');
        
    case '4dof-scara' % A-2084-01
        %%
        % Create group for communications
        group = HebiLookup.newGroupFromNames(family, {
            'Base'
            'Shoulder'
            'Elbow'
            'Wrist1' });
        
        % Kinematic Model
        kin = HebiKinematics();
        kin.addBody('X5-4');
        kin.addBody('X5-HeavyBracket', 'mount', 'right-outside');
        kin.addBody('X5-9');
        kin.addBody('X5-Link', 'extension', 0.325, 'twist', pi/2);
        kin.addBody('X5-4');
        kin.addBody('X5-Link', 'extension', 0.325, 'twist', 0);
        kin.addBody('X5-1');
        
    case '3-DoF' % A-2084-01
        
        group = HebiLookup.newGroupFromNames(family, {
            'Base'
            'Shoulder'
            'Elbow' } );
        
        kin = HebiKinematics('/hrdf/3-DoF_arm_example');

    otherwise
        error([kit ' is not a supported kit name']);
        
end


%% Common Setup
% Use default effort offsets
if ~exist('effortOffset', 'var') || isempty(effortOffset)
    params.effortOffset = zeros(1, kin.getNumDoF);
end

% Determine initial gravity vector based on the internal pose filter of
% the base module
fbk = group.getNextFeedbackFull();
baseRotMat = HebiUtils.quat2rotMat( [ 
    fbk.orientationW(1), ...
    fbk.orientationX(1), ...
    fbk.orientationY(1), ...
    fbk.orientationZ(1) ] );
params.gravityVec = -baseRotMat(3,1:3);

% If the base module does not report a quaternion (e.g. old firmware), then
% fall back to using the accelerometer. This assumes that the base is idle
% and that the accelerometers are reading only gravity.
if any(isnan(params.gravityVec))
    gravityVec = -[fbk.accelX(1) fbk.accelY(1) fbk.accelZ(1)];
    params.gravityVec = gravityVec / norm(gravityVec);
end

end

