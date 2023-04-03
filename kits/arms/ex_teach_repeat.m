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

% Copyright 2017-2018 HEBI Robotics

%% Setup

% Reset the workspace
clear *;
close all;

armName = 'A-2240-06';
armFamily = 'Arm';
hasGasSpring = false;  % If you attach a gas spring to the shoulder for
                       % extra payload, set this to TRUE.

[ arm, params ] = setupArm( armName, armFamily, hasGasSpring );
arm.group.setFeedbackFrequency(100);
arm.trajGen.setMinDuration(1.0);   % Min move time for 'small' movements
                                   % (default is 1.0)
arm.trajGen.setSpeedFactor(0.75);  % Slow down movements to a safer speed.
                                   % (default is 1.0)
arm.plugins = {
    HebiArmPlugins.EffortOffset(params.effortOffset)
};

% Keyboard input
kb = HebiKeyboard();
localDir = params.localDir;

% === Demo configuration ===
% Select whether waypoints should be done as a single trajectory, or
% multiple trajectories that stop in between.
stopBetweenWaypoints = true;

% Select whether you want to log and visualize the replay movement
enableLogging = true;

%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Record waypoints in gravity compensated mode %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Move the arm to different positions to set waypoints.');
disp('  ALT  - Adds a new waypoint.');  
disp('  ESC  - Exits waypoint training mode.');
disp('         If no waypoints are set, default waypoints are loaded.');
disp('  ');


waypoints = [];
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

numWaypoints = size(waypoints,1);
if numWaypoints == 0
    % Load the set of default waypoints for a 6-DoF arm and trim them down 
    % to the proper number of DoF.
    waypointDir = [localDir '/waypoints/'];
    waypointFileName = 'defaultWaypoints';
    
    tempStruct = load( [waypointDir waypointFileName] );
    waypoints = tempStruct.waypoints(:,1:arm.kin.getNumDoF());
    
    disp('  '); 
    disp('No waypoints saved.  Loading default waypoints.');  
else
    waypointDir = [localDir '/waypoints'];
    waypointFileName = 'latestWaypoints';
    save([waypointDir '/' waypointFileName], 'waypoints');
    
    disp( '  ' );
    disp( [ num2str(numWaypoints) ' waypoints saved.' ] );  
end
disp( 'Press SPACE to move to first waypoint.' );

% Stay in grav-comp mode to prevent sudden movements from effort commands 
% turning on and off.
while keys.SPACE == 0
    arm.update();
    arm.send();
    keys = read(kb);
end

%% 
%%%%%%%%%%%%%%%%%%%%
% Replay waypoints %
%%%%%%%%%%%%%%%%%%%%

% Start background logging 
if enableLogging
   logFile = arm.group.startLog( 'dir', [localDir '/logs'] ); 
end

% Move from current position to first waypoint
arm.update();
arm.clearGoal(); % in case we re-run this section
arm.setGoal(waypoints(1,:));
abortFlag = false;
while ~arm.isAtGoal() && ~abortFlag
   arm.update();
   arm.send();
   
   % Check for keyboard input and break out of the main loop
   % if the ESC key is pressed.
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

disp('Beginning playback.');
disp('Press ESC to stop.');

% Move along waypoints in a loop
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

disp('  ');
disp('Quitting playback.');

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Stop background logging and visualize %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if enableLogging
    
   disp('Plotting logged feedback.');
   hebilog = arm.group.stopLogFull();
   
   % Plot tracking / error from the joints in the arm
   HebiUtils.plotLogs(hebilog, 'position');
   HebiUtils.plotLogs(hebilog, 'velocity');
   HebiUtils.plotLogs(hebilog, 'effort');
   
   % Plot the end-effectory trajectory and error
   kinematics_analysis( hebilog, arm.kin );
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   % Feel free to put more plotting code here %
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

disp('DONE.');

