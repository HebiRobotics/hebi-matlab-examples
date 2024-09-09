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

% Copyright 2017-2018 HEBI Robotics

%% Setup
clear *;
close all;

HebiLookup.initialize();

enableLogging = true;

%% Load config file
localDir = fileparts(mfilename('fullpath'));
exampleConfigFile = fullfile(localDir, 'config', 'ex_AR_kit.cfg.yaml');
exampleConfig = HebiUtils.loadRobotConfig(exampleConfigFile);

%%
%%%%%%%%%%%%%
% Arm Setup %
%%%%%%%%%%%%%

% Instantiate the arm kit based on the config files in config/${name}.yaml
% If your kit has a gas spring, you need to uncomment the offset lines
% in the corresponding config file.
arm = createArmFromConfig(exampleConfig);

% Demo Variables
abortFlag = false;
runMode = "softstart";
% goal = hebi.arm.Goal(arm.size)

% Command the softstart to the home position
arm.update();
arm.clearGoal(); % in case we run only this section
arm.setGoal(exampleConfig.userData.home_position, ...
            'time', exampleConfig.userData.homing_duration);
while ~arm.isAtGoal
    arm.update();
    arm.send();
end

% Get the cartesian position and rotation matrix @ home position
transformHome = arm.kin.getFK('endEffector', exampleConfig.userData.home_position);
xyzHome = transformHome(1:3, 4);
rotHome = transformHome(1:3, 1:3);
                             
% disp('  ');
% disp('Arm end-effector can now follow the mobile device pose in AR mode.');
% disp('The control interface has the following commands:');
% disp('  🏠 - Home');
% disp('       This takes the arm to home and aligns with mobile device.');
% disp('  📲 - Engage/Disengage AR Control');
% disp('  🌍 - Go to grav comp mode.');
% disp('  ❌ - Quit the demo.');

% Print instructions
instructions = [
'Arm end-effector can now follow the mobile device pose in AR mode.', ...
'The control interface has the following commands:', ...
'  🏠 - Home', ...
'       This takes the arm to home and aligns with mobile device.', ...
'  📲 - Engage/Disengage AR Control', ...
'  🌍 - Go to grav comp mode.', ...
'  ❌ - Quit the demo.'
]; 

disp(instructions);

%%
%%%%%%%%%%%%%%%%%%%%%%%
% Mobile Device Setup %
%%%%%%%%%%%%%%%%%%%%%%%

mobileIO = createMobileIOFromConfig(exampleConfig);
mobileIO.update();

% Start background logging
if enableLogging
    arm.group.startLog('dir',[localDir '/logs']);
end

%% Startup
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
    if fbkMobileIO.('b1')
        runMode = "waiting";
        arm.setGoal(exampleConfig.userData.home_position, ...
                    'time', exampleConfig.userData.homing_duration);
    end

    % Check for AR Mode command
    if fbkMobileIO.('b3') && runMode ~= "ar_mode" % ToOn

        runMode = "ar_mode";

        % Store initial position and orientation as baseline
        rotPhone_init = mobileIO.getArOrientation();
        xyzPhone_init = mobileIO.getArPosition();

    end
    
    % Check for Grav Comp
    if fbkMobileIO.('b6')
        runMode = "grav_comp";
        arm.clearGoal();
    end

    % Check for quit command
    if fbkMobileIO.('b8')
        abortFlag = true;
        break;
    end

    if runMode == "ar_mode"

        rotPhone = mobileIO.getArOrientation();
        xyzPhone = mobileIO.getArPosition();

        xyzTarget = xyzHome + ...
            exampleConfig.userData.xyz_scale * [1; 1; 2] .* (rotPhone_init' * (xyzPhone - xyzPhone_init));   
        rotTarget = rotPhone_init' * rotPhone * rotHome;

        % Use inverse kinematics to calculate appropriate joint positions
        targetJoints = arm.kin.getIK('XYZ', xyzTarget, ...
            'SO3', rotTarget, ...
            'initial', arm.group.getNextFeedback().position);

        % Set snapped pose as goal
        % arm.clearGoal();
        arm.setGoal(targetJoints);
    end
end

%%
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

