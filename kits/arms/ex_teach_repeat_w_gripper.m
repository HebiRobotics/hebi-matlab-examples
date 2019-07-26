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

[ arm, armParams ] = setupArm( armName, armFamily, hasGasSpring );
arm.group.setFeedbackFrequency(100);

effortOffset = armParams.effortOffset;
localDir = armParams.localDir;

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

arm.gripper.open(); % initialize gripper state to open
arm.enableCommands = false; % disables position / velocities / accel-comp-efforts

waypoints = [];
gripStates = [];

keys = read(kb);
while keys.ESC == 0
    
    % Do grav-comp while training waypoints
    [cmd, state] = arm.update();
    cmd.effort = cmd.effort + effortOffset;  
    arm.send(cmd);
                        
    % Add new waypoints 
    [keys, diffKeys] = read(kb);
    
    % Toggle gripper on CTRL key press
    if diffKeys.CTRL == 1 
        disp('Toggling Gripper.');
        arm.gripper.toggle();
    end
    
    % Store waypoints on CTRL or ALT press
    if diffKeys.CTRL == 1 || diffKeys.ALT == 1
        waypoints(end+1,:) = state.fbk.position;
        gripStates(end+1,:) = arm.gripper.state;
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
    gripStates = tempStruct.gripStates;
    
    disp('  '); 
    disp('No waypoints saved.  Loading default waypoints.');  
else
    
    waypointDir = [localDir '/waypoints'];
    waypointFileName = 'latestWaypoints';
    save([waypointDir '/' waypointFileName], 'waypoints', 'gripStates');
    
    disp( '  ' );
    disp( [ num2str(numWaypoints) ' waypoints saved.' ] );  
end
disp( 'Press SPACE to move to first waypoint.' );

% Stay in grav-comp mode to prevent sudden movements from effort commands 
% turning on and off.
while keys.SPACE == 0
    [cmd, state] = arm.update();
    cmd.effort = cmd.effort + effortOffset;
    arm.send(cmd);
    keys = read(kb);
end

abortFlag = false;

%% 
%%%%%%%%%%%%%%%%%%%%
% Replay waypoints %
%%%%%%%%%%%%%%%%%%%%

% Start background logging 
if enableLogging
   logFile = arm.group.startLog( 'dir', [localDir '/logs'] ); 
end

% Move from current position to first waypoint
arm.enableCommands = true;
arm.initialize(); % TODO: warm startup! 
arm.setGoal(waypoints(1,:));

% Execute the trajectory to the first waypoint
disp('  '); 
disp('Ready to begin playback.');
disp('Press SPACE again to begin.');

while true

    [cmd, state] = arm.update();
    cmd.effort = cmd.effort + effortOffset;
    arm.send(cmd);
   
    % Check for keyboard input and break out of the main loop
    % if the ESC key is pressed.  
    keys = read(kb);    
    if keys.ESC == 1
        abortFlag = true;
        break;
    end
    
    if arm.isAtGoal() && keys.SPACE == 1
       break; 
    end
    
end

% Hang out at the first waypoint until we press SPACE
disp('Beginning playback.');
disp('Press ESC to stop.');  

%%%%%%%%%%%%%%%%%%%%%%%%
% Move along waypoints %
%%%%%%%%%%%%%%%%%%%%%%%%
while ~abortFlag
    
    arm.setGoal(waypoints, 'velocities', zeros(size(waypoints)), 'accelerations', zeros(size(waypoints))); 
    while ~arm.isAtGoal()
        
        cmd = arm.update();
        cmd.effort = cmd.effort + effortOffset;
        arm.send(cmd);
        
        [keys] = read(kb);
        if keys.ESC == 1
            abortFlag = true;
            break;
        end
        
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
   kinematics_analysis( hebilog, armKin );
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   % Feel free to put more plotting code here %
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

disp('DONE.');

