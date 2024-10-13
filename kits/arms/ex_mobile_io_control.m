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

% Copyright 2017-2024 HEBI Robotics

%% Setup
clear *;
close all;
HebiLookup.initialize();

% Demo Settings
enableLogging = true;

%% Load config and setup components
config = HebiUtils.loadRobotConfig('config/ex_mobile_io_control.cfg.yaml');
userData = config.userData;
arm = HebiArm.createFromConfig(config);
mobileIO = createMobileIOFromConfig(config);

%% Start optional background logging
if enableLogging
    logFile = arm.group.startLog('dir', 'logs');
end

%% Demo
disp([
'   B1-B3      - Waypoints 1-3', newline ...
'(Earth Emoji) - Grav Comp Mode', newline ...
'(Cross Emoji) - Quit'
]); 

abortFlag = false;
while ~abortFlag

    % Update the state
    arm.update();
    arm.send();

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

    % B1-3 Move to predefined waypoints
    if fbkMobileIO.b1
        arm.setGoal(userData.waypoint_1, 'time', userData.travel_time);
    elseif fbkMobileIO.b2
        arm.setGoal(userData.waypoint_2, 'time', userData.travel_time);
    elseif  fbkMobileIO.b3
        arm.setGoal(userData.waypoint_3, 'time', userData.travel_time);
    end

    % B6 - Grav Comp
    if fbkMobileIO.b6
        arm.clearGoal();
    end

    % B8 - Quit
    if fbkMobileIO.b8
        abortFlag = true;
    end
    
end

disp('Stopping Demo...')

%% Stop Logging
if enableLogging  
   hebilog = arm.group.stopLogFull();
end

%%
% Plotting
if enableLogging
   
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

