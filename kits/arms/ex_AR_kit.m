% 6-DoF Arm (w/ Optional Gripper) Tele-Op Demo
%
% Features:      Mobile App input to control motions of arm and gripper
%
% Requirements:  MATLAB 2013b or higher
%                Android or iOS device running HEBI Mobile I/O that
%                supports ARCore / ARKit.
%
% Author:        Dave Rollinson
%
% Date:          Oct 2018

% Copyright 2017-2024 HEBI Robotics

%% Setup
clear *;
close all;
HebiLookup.initialize();

% Demo Settings
enableLogging = true;

%% Set up arm, and mobileIO from config
config = HebiUtils.loadRobotConfig('config/ex_AR_kit.cfg.yaml');
userData = config.userData;
arm = HebiArm.createFromConfig(config);
mobileIO = createMobileIOFromConfig(config);

%% Homing / Initialization
% Command the softstart to the home position
arm.update();
arm.clearGoal(); % in case we run only this section
arm.setGoal(userData.home_position, ...
    'time', userData.homing_duration);

% Get the cartesian position and rotation matrix @ home position
transformHome = arm.kin.getFK('endEffector', userData.home_position);
xyzHome = transformHome(1:3, 4);
rotHome = transformHome(1:3, 1:3);

xyzScale = userData.xyz_scale';

% Print instructions
instructions = [
    'Arm end-effector can now follow the mobile device pose in AR mode.', newline ...
    'The control interface has the following commands:', newline ...
    '  (House emoji) - Home', newline ...
    '                  This takes the arm to home and aligns with mobile device.', newline ...
    '  (Phone emoji) - Engage/Disengage AR Control', newline ...
    '  (Earth emoji) - Go to grav comp mode.', newline ...
    '  (Cross emoji) - Quit the demo.'
    ];

disp(instructions);

%% Start optional background logging
if enableLogging
    logFile = arm.group.startLog('dir', 'logs');
end

%% Demo
runMode = "softstart";
abortFlag = false;
while ~abortFlag

    %%%%%%%%%%%%%%%%%%%
    % Gather Feedback %
    %%%%%%%%%%%%%%%%%%%
    try
        arm.send()
        arm.update();
    catch
        disp('Could not get robot feedback!');
        break;
    end

    if runMode == "softstart"
        % End softstart when the arm reaches the home_position
        if arm.isAtGoal
            runMode = "waiting";
            continue;
        end
        continue;
    end

    % We use a non-blocking timeout w/ manual update checks so we can
    % handle common wireless delays/outages without breaking. We only
    % exit if mobileIO hasn't responded for several seconds, which
    % indicates a more significant problem.
    [hasNewMobileFbk, timeSinceLastFeedback] = mobileIO.update('timeout', 0);
    if timeSinceLastFeedback > 5
        error('mobileIO has timed out. Stopping demo.');
    end

    % Abort goal updates if the phone didn't respond
    if ~hasNewMobileFbk
        continue;
    end
    fbkMobileIO = mobileIO.getFeedbackIO();

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Read/Map Joystick Inputs %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Check for homing command
    if fbkMobileIO.b1
        runMode = "waiting";
        arm.setGoal(userData.home_position, ...
            'time', userData.homing_duration);
    end

    % Check for AR Mode command
    if fbkMobileIO.b3 && runMode ~= "ar_mode" % ToOn
        runMode = "ar_mode";

        % Store initial position and orientation as baseline
        rotPhone_init = mobileIO.getArOrientation();
        xyzPhone_init = mobileIO.getArPosition();

    end

    % Check for Grav Comp
    if fbkMobileIO.b6
        runMode = "grav_comp";
        arm.clearGoal();
    end

    % Check for quit command
    if fbkMobileIO.b8
        abortFlag = true;
        break;
    end

    if runMode == "ar_mode"

        rotPhone = mobileIO.getArOrientation();
        xyzPhone = mobileIO.getArPosition();

        xyzTarget = xyzHome + ...
            xyzScale .* (rotPhone_init' * (xyzPhone - xyzPhone_init));
        rotTarget = rotPhone_init' * rotPhone * rotHome;

        % Use inverse kinematics to calculate appropriate joint positions
        targetJoints = arm.kin.getIK( 'XYZ', xyzTarget, ...
                                      'SO3', rotTarget, ...
                                      'initial', arm.state.fbk.position);
        
        % Sanity check (uncomment for debug info)
%         T_check = arm.kin.getFK( 'endeffector', targetJoints );
%         xyz_check = T_check(1:3,4);
%         xyzTarget - xyz_check

        % Set snapped pose as goal
        arm.setGoal( targetJoints, 'time', userData.delay_time );
    end
end

disp('Quitting Demo.');

%% Analysis of logged data
if enableLogging
    
    fprintf('Loading log and plotting...');
    hebilog = arm.group.stopLogFull();

    % Plot tracking / error from the joints in the arm.  Note that there
    % will not by any 'error' in tracking for position and velocity, since
    % this example only commands effort.
    HebiUtils.plotLogs(hebilog, 'position');
    HebiUtils.plotLogs(hebilog, 'velocity');
    HebiUtils.plotLogs(hebilog, 'effort');

    % Plot the end-effectory trajectory and error
    kinematics_analysis( hebilog, arm.kin );

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Feel free to put more plotting code here %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    fprintf('DONE.\n\n');
end

