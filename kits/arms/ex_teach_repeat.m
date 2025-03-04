% Arm Teach-Repeat Demo
%
% Features:      Demo with two modes.  One for moving the arm to different
%                waypoints while in a zero-force gravity-compensated mode.
%                And a second mode for playing back the waypoints.
%
% Requirements:  MATLAB 2013b or higher
%
% Author:        Dave Rollinson
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
% Date:          Oct 2018

% Copyright 2017-2024 HEBI Robotics

%% Setup
clear *;
close all;
HebiLookup.initialize();

% Select whether waypoints should be done as a single trajectory, or
% multiple trajectories that stop in between.
stopBetweenWaypoints = true;

% Select whether you want to log and visualize the replay movement
enableLogging = true;

%% Load config and setup components
config = HebiUtils.loadRobotConfig('config/ex_teach_repeat.cfg.yaml');
userData = config.userData;
arm = HebiArm.createFromConfig(config);

% Set a minimum move time for 'small' movements. Default is 1.0
arm.trajGen.setMinDuration(userData.min_travel_time);

%% Step 1 - Record waypoints in gravity compensated mode
disp('Move the arm to different positions to set waypoints.');
disp('  ALT  - Adds a new waypoint.');  
disp('  ESC  - Exits waypoint training mode.');
disp('  ');

waypoints = [];

kb = HebiKeyboard();
keys = read(kb);
while keys.ESC == 0
    
    % Do grav-comp while training waypoints
    arm.update();
    arm.send();

    % Add new waypoints on ALT press
    [keys, diffKeys] = read(kb);
    if diffKeys.ALT == 1    
        waypoints(end+1,:) = arm.state.fbk.position;
        disp('Waypoint added.');
    end
    
end

% Make sure we have enough points
numWaypoints = size(waypoints,1);
if numWaypoints < 2
    disp('Demo aborted because you selected fewer than two waypoints.');
    return;
end

%% Wait for input
% Stay in grav-comp mode to prevent sudden movements from effort commands 
% turning on and off.
disp( 'Press SPACE to move to first waypoint.' );
while keys.SPACE == 0
    arm.update();
    arm.send();
    keys = read(kb);
end

%% Start optional background logging
if enableLogging
    logFile = arm.group.startLog('dir', 'logs');
end

%% Step 2 - Replay waypoints

% Move from current position to first waypoint
arm.clearGoal(); % in case we re-run this section
arm.update();
arm.setGoal(waypoints(1,:));
abortFlag = false;
while ~arm.isAtGoal() && ~abortFlag
   arm.update();
   arm.send();
   keys = read(kb);
   abortFlag = keys.ESC;
end

% Hang out at the first waypoint until we press SPACE
disp('  '); 
disp('Ready to begin playback.');
disp('Press SPACE to begin.');
while keys.SPACE == 0
    arm.update();
    arm.send();
    keys = read(kb);
end

% Move along waypoints in a loop
disp('Beginning playback.');
disp('Press ESC to stop.');
while ~abortFlag
    
    if ~stopBetweenWaypoints
        % Moves through all waypoints in one movement
        arm.setGoal(waypoints);
    else
        % Adds zero velocity/acceleration constraints at each
        % waypoint to force a short stop
        arm.setGoal(waypoints, ...
            'velocities', 0 * waypoints, ...
            'accelerations', 0 * waypoints);
    end
    
    % Execute motion
    while ~arm.isAtGoal() && ~abortFlag
       arm.update();
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       % If you want to do something with the latest feedback to
       % change the commands, replan a trajectory, abort, or do
       % anything else, this is a pretty good place to do it.
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        arm.send();
        keys = read(kb);
        abortFlag = keys.ESC;
    end
    
end

disp('Stopping Demo.')

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

