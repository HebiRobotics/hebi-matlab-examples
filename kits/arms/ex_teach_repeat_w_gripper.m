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

[ armGroup, armKin, armParams ] = setupArm( armName, armFamily, hasGasSpring );
armGroup.setFeedbackFrequency(100);

arm = ;% ...

numDoF = armKin.getNumDoF();

effortOffset = armParams.effortOffset;
gravityVec = armParams.gravityVec;
localDir = armParams.localDir;

if armParams.hasGripper
    gripperGroup = HebiLookup.newGroupFromNames( armFamily, 'Spool' );
    gripperGroup.send( 'gains', armParams.gripperGains );
    gripperOpenEffort = armParams.gripperOpenEffort;
    gripperCloseEffort = armParams.gripperCloseEffort;
    gripperCmd = CommandStruct();
end

% Trajectory
trajGen = HebiTrajectoryGenerator(armKin);
trajGen.setMinDuration(1.0); % Min move time for 'small' movements
                             % (default is 1.0)
trajGen.setSpeedFactor(0.75); % Slow down movements to a safer speed.
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

waypoints = [];
gripStates = 0;  % 0 is open, 1 is closed
                % Initialize to open, then we'll trim the first waypoint
                % off afterwards.

keys = read(kb);
prevKeys = keys;

cmd = CommandStruct();

% TODO: we need to do an arm initialize() or arm.moveHome() w/ gain
% ramp-up/soft-start
% arm.setEnableGravComp(true); % defaults to true
arm.setActiveCommands(false); % disables position / velocities / accel-comp-efforts
while keys.ESC == 0
    
    % Do grav-comp while training waypoints
    cmd = arm.update(); % ... internal getNextFeedback, pre-calculate Jacobians, 
                        % end effector positions/velocities, etc.
    cmd.effort = cmd.effort + effortOffset;    

    % Add new waypoints 
    keys = read(kb);
    
    % Add 'Move' waypoint on ALT key press
    if keys.ALT == 1 && prevKeys.ALT == 0      
        waypoints(end+1,:) = arm.fbk.position;
        gripStates(end+1,1) = arm.gripper.state; % Keep gripper state the same
        disp('Waypoint added.');
    end
    
    % Add 'Gripper' waypoint on CTRL key press
    if keys.CTRL == 1 && prevKeys.CTRL == 0      
        arm.gripper.toggle();
        waypoints(end+1,:) = arm.fbk.position;
        gripStates(end+1,1) = arm.gripper.state; % Keep gripper state the same
        disp('Waypoint added.  Toggling Gripper.');
    end
    
%     arm.kin.getIK(arm.fbk.position, varargin{:}); % wrapper that forwards seed angles?
    
    % Keep track of last key presses for diffing
    prevKeys = keys;
    
    % internally: arm.group.send(varargin{:}), gripper.set(gripperState)
    arm.send(cmd); % punting on gripper stuff for now % needs to be ramped up? scale gains?
    
end

% wpts = HebiWaypoints();
% wpts.add(fbk.position, aux.state);
% List<struct<armState, auxState>> waypoints;

numWaypoints = size(waypoints,1);

if numWaypoints == 0
    % Load the set of default waypoints for a 6-DoF arm and trim them down 
    % to the proper number of DoF.
    waypointDir = [localDir '/waypoints/'];
    waypointFileName = 'defaultWaypoints';
    
    tempStruct = load( [waypointDir waypointFileName] );
    waypoints = tempStruct.waypoints(:,1:numDoF);
    gripStates = tempStruct.gripStates;
    
    disp('  '); 
    disp('No waypoints saved.  Loading default waypoints.');  
else
    % Trim the first waypoint off of gripStates that we used to make
    % everything initialize right.
    gripStates(1) = [];
    
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
    cmd = arm.update();
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
   logFile = arm.armGroup.startLog( 'dir', [localDir '/logs'] ); 
end

% Move from current position to first waypoint
arm.setEnableCommands(true); % TODO: warm startup! go to current feedback
%arm.setGoal('positions', pos, 'velocities', vel, 'accelerations', accel, 'time', time); % Calculates trajectory from current pos (fbk) to target
% same as newJointMove, but includes current state? [cmdPos pos] => traj
% generator (or fbkPos)
arm.setGoal('positions', waypoints(1,:), 'aux', auxWaypoints(1,:));

% Execute the trajectory to the first waypoint
% arm.setEnableDynamicsComp(true); % defaults to true

disp('  '); 
disp('Ready to begin playback.');
disp('Press SPACE again to begin.');

while true

    cmd = arm.update();
    cmd.effort = cmd.effort + effortOffset;
    arm.send(cmd);
   
    % Check for keyboard input and break out of the main loop
    % if the ESC key is pressed.  
    keys = read(kb);    
    if keys.ESC == 1
        abortFlag = true;
        break;
    end
    
    % ... text ... press space bar to continue
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
    
    % TODO: remove duplicate waypoints from auto-timing
    arm.setGoal('position', waypoints, 'velocities', zeros(size(waypoints))); 
    while arm.getProgress() < 1 % and abort flag
        cmd = arm.update();
        
        efforts = springCompensation.getEfforts(arm);
        
        cmd.effort = cmd.effort + effortOffset;
        arm.send(cmd);
        
        [keys diffKeys] = read(kb);
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
if arm.group.isLogging() % add to API
    
   hebilog = armGroup.stopLogFull();
   
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

