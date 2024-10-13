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
xyzScale = [1 1 2]';

%% Set up arm, mobileIO, and gripper from config
config = HebiUtils.loadRobotConfig('config/ex_AR_kit_w_gripper.cfg.yaml');
userData = config.userData;
arm = HebiArm.createFromConfig(config);
mobileIO = createMobileIOFromConfig(config);
gripper = createGripperFromConfig(config);

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

% Print instructions
instructions = [
    'Arm end-effector can now follow the mobile device pose in AR mode.', newline ...
    'The control interface has the following commands:', newline ...
    '  (Hand emoji)  - Gripper Control' ...
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
        arm.send();
        arm.update();
        gripper.send();
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
            userData.xyz_scale * xyzScale .* (rotPhone_init' * (xyzPhone - xyzPhone_init));
        rotTarget = rotPhone_init' * rotPhone * rotHome;

        % Use inverse kinematics to calculate appropriate joint positions
        targetJoints = arm.kin.getIK('XYZ', xyzTarget, ...
            'SO3', rotTarget, ...
            'initial', arm.group.getNextFeedback().position);

        % Set snapped pose as goal
        arm.setGoal(targetJoints);
    end


    % Set the gripper separately to follow slider A3
    % Map [-1,+1] slider to [0,1] range such that down is close
    gripper.setState((fbkMobileIO.a3 - 1) / -2);

end

%% Analysis of logged data
if enableLogging

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
end

