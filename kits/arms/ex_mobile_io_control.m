% 6-DoF Arm (w/ Optional Gripper) Tele-Op Demo 
%
% Features:      Mobile App input to control motions of arm
%
% Requirements:  MATLAB 2013b or higher
%                Android or iOS device running HEBI Mobile I/O
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
exampleConfigFile = fullfile(localDir, 'config', 'ex_mobile_io_control.cfg.yaml');
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
% arm.clearGoal(); % in case we run only this section
% arm.setGoal(exampleConfig.userData.home_position, ...
%             'time', exampleConfig.userData.homing_duration);

% Print instructions
instructions = [
'   B1-B3      - Waypoints 1-3', newline ...
'(Earth Emoji) - Grav Comp Mode', newline ...
'(Cross Emoji) - Quit'
]; 

disp(instructions);

%%
%%%%%%%%%%%%%%%%%%%%%%%
% Mobile Device Setup %
%%%%%%%%%%%%%%%%%%%%%%%

mobileIO = createMobileIOFromConfig(exampleConfig);
mobileIO.update();

buttons = ['b1'; 'b2'; 'b3'];
waypoints = [exampleConfig.userData.waypoint_1; exampleConfig.userData.waypoint_2; exampleConfig.userData.waypoint_3];

% Start background logging
if enableLogging
    arm.group.startLog('dir',[localDir '/logs']);
end

%% Startup
while ~abortFlag
    % Update the arm
    arm.update();

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

    % Iterate over buttons 1, 2, 3
    for N = 1:3
        % BN - Waypoint N (N = 1, 2, 3)
        if fbkMobileIO.(buttons(N, :))  % "ToOn"
            arm.setGoal(waypoints(N,:), 'time', exampleConfig.userData.travel_time);
        end
    end

    % B6 - Grav Comp
    if fbkMobileIO.('b6')  % "ToOn"
        arm.clearGoal();
    end

    % B8 - Quit
    if fbkMobileIO.('b8')  % "ToOn"
        % Reset text & color, and quit
        abortFlag = true;
        break;
    end

    % Send the arm command
    arm.send();
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

