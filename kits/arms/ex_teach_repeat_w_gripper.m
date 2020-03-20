% Arm Teach-Repeat w/ Gripper Demo
%
% Features:      Demo with two modes.  One for moving the arm to different
%                waypoints while in a zero-force gravity-compensated mode.
%                And a second mode for playing back the waypoints.  In this
%                demo 'grip waypoints' can also be set that toggle a
%                gripper between being open and closed.
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

armName = 'A-2085-06G';
armFamily = 'Arm';
hasGasSpring = false;  % If you attach a gas spring to the shoulder for
                       % extra payload, set this to TRUE.

[ arm, params, gripper ] = setupArm( armName, armFamily, hasGasSpring );
arm.group.setFeedbackFrequency(100);
arm.plugins = {
    HebiArmPlugins.EffortOffset(params.effortOffset)
};
arm.trajGen.setMinDuration(1.0);   % Min move time for 'small' movements
                                   % (e.g. gripper open/close)
arm.trajGen.setSpeedFactor(0.75);  % Slow down movements to a safer speed.
                                   % (default is 1.0)

% Keyboard input
kb = HebiKeyboard();

% Select whether you want to log and visualize the replay movement
enableLogging = true;


%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Record waypoints in gravity compensated mode %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Move the arm to different positions to set waypoints.');
disp('  ALT  - Adds a new waypoint.');  
disp('  CTRL - Adds a new waypoint and toggles the gripper.');  
disp('  ESC  - Exits waypoint training mode.');
disp('         If no waypoints are set, default waypoints are loaded.');
disp('  ');

gripper.open(); % initialize gripper state to open
waypoints = [];
gripStates = [];

keys = read(kb);
while keys.ESC == 0
    
    % Do grav-comp while training waypoints
    arm.update();
    arm.send();
    gripper.send();
                        
    % Add new waypoints 
    [keys, diffKeys] = read(kb);
        
    % Add 'toggling' waypoint on CTRL press
    % (shows up as 2 waypoints to allow
    % the gripper to open or close)
    if diffKeys.CTRL == 1
        disp('Toggling Gripper.');
        gripper.toggle();
        waypoints(end+1,:) = arm.state.fbk.position;
        gripStates(end+1,:) = gripper.state;
    end
    
    % Store waypoints on CTRL or ALT press
    if diffKeys.CTRL == 1 || diffKeys.ALT == 1
        waypoints(end+1,:) = arm.state.fbk.position;
        gripStates(end+1,:) = gripper.state;
        disp('Waypoint added.');
    end

end

numWaypoints = size(waypoints,1);
if numWaypoints == 0
    % Load the set of default waypoints for a 6-DoF arm and trim them down 
    % to the proper number of DoF.
    waypointDir = [params.localDir '/waypoints/'];
    waypointFileName = 'defaultWaypoints';
    
    tempStruct = load( [waypointDir waypointFileName] );
    waypoints = tempStruct.waypoints(:,1:arm.kin.getNumDoF());
    gripStates = tempStruct.gripStates;
    
    disp('  '); 
    disp('No waypoints saved.  Loading default waypoints.');  
else
    
    waypointDir = [params.localDir '/waypoints'];
    waypointFileName = 'latestWaypoints';
    save([waypointDir '/' waypointFileName], 'waypoints', 'gripStates');
    
    disp( '  ' );
    disp( [ num2str(numWaypoints) ' waypoints saved.' ] );  
end
disp( 'Press SPACE to move to first waypoint.' );

% Stay in grav-comp mode to prevent sudden movements from effort commands 
% turning on and off.
while keys.SPACE == 0
    arm.update();
    arm.send();
    gripper.send();
    keys = read(kb);
end

%% 
%%%%%%%%%%%%%%%%%%%%
% Replay waypoints %
%%%%%%%%%%%%%%%%%%%%
abortFlag = false;

% Start background logging 
if enableLogging
   logFile = arm.group.startLog( 'dir', [params.localDir '/logs'] ); 
end

% Move from current position to first waypoint
arm.clearGoal(); % if you run in sections ...
arm.update(); % muste be called first
arm.setGoal(waypoints(1,:));

% Execute the trajectory to the first waypoint
disp('  '); 
disp('Ready to begin playback.');
disp('Press SPACE again to begin.');

keys = read(kb);
while ~arm.isAtGoal() || keys.SPACE == 0 && ~abortFlag

    arm.update();
    arm.send();
   
    % Check for keyboard input and break out of the main loop
    % if the ESC key is pressed.  
    keys = read(kb);   
    abortFlag = keys.ESC;
    
end

% Hang out at the first waypoint until we press SPACE
disp('Beginning playback.');
disp('Press ESC to stop.');  

%%%%%%%%%%%%%%%%%%%%%%%%
% Move along waypoints %
%%%%%%%%%%%%%%%%%%%%%%%%
while ~abortFlag
    
    % Add zero constraints to stop at each waypoint
    arm.setGoal(waypoints, ...
        'velocities', 0 * waypoints, ...
        'accelerations', 0 * waypoints, ...
        'aux', gripStates);
    
    while ~arm.isAtGoal() && ~abortFlag
        arm.update();
        arm.send();
        gripper.setState(arm.state.cmdAux);
        gripper.send();
        abortFlag = read(kb).ESC;
    end

end

disp('  ');
disp('Quitting playback.');
disp('Plotting logged feedback.');


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Stop background logging and visualize %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if enableLogging
    
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

