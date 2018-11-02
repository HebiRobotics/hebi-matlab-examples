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

armName = '6-DoF + gripper';
armFamily = 'Arm';
hasGasSpring = true;

[ armGroup, armKin, armParams ] = setupArm( armName, armFamily, hasGasSpring );
armGroup.setFeedbackFrequency(100);

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

while keys.ESC == 0
    
    % Do grav-comp while training waypoints
    fbk = armGroup.getNextFeedback();
    cmd.effort = armKin.getGravCompEfforts(fbk.position, gravityVec) ...
                                                        + effortOffset;
    armGroup.send(cmd);
    
    % Command the gripper to the most recent state
    if gripStates(end,1)==1
        gripperCmd.effort = gripperCloseEffort;
    else
        gripperCmd.effort = gripperOpenEffort;
    end
    gripperGroup.send(gripperCmd);
    
    % Add new waypoints 
    keys = read(kb);
    
    % Add 'Move' waypoint on ALT key press
    if keys.ALT == 1 && prevKeys.ALT == 0      
        waypoints(end+1,:) = fbk.position;
        gripStates(end+1,1) = gripStates(end,1); % Keep gripper state the same
        disp('Waypoint added.');
    end
    
    % Add 'Gripper' waypoint on CTRL key press
    if keys.CTRL == 1 && prevKeys.CTRL == 0      
        waypoints(end+1,:) = fbk.position;
        gripStates(end+1,1) = ~gripStates(end,1); % Keep gripper state the same
        disp('Waypoint added.  Toggling Gripper.');
    end
    
    % Keep track of last key presses for diffing
    prevKeys = keys;
    
end

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
    fbk = armGroup.getNextFeedback();
    cmd.effort = armKin.getGravCompEfforts(fbk.position, gravityVec) ...
                                                        + effortOffset;
    armGroup.send(cmd);
    keys = read(kb);
end

abortFlag = false;

%% 
%%%%%%%%%%%%%%%%%%%%
% Replay waypoints %
%%%%%%%%%%%%%%%%%%%%

% Start background logging 
if enableLogging
   logFile = armGroup.startLog( 'dir', [localDir '/logs'] ); 
end

% Move from current position to first waypoint
% This uses the blocking API, which means you can't easily look at 
% feedback while it is exectuting.
startPosition = armGroup.getNextFeedback().position;
endPosition = waypoints(1,:);
movePositions = [ startPosition;
                  endPosition ];
              
% Make a new point-to-point trajectory and update the offset
% for timing the trajectory.  This uses the non-blocking
% trajectory API so that we can easily look at feedback from
% from the arm while it is running.
trajectory = trajGen.newJointMove( movePositions );
trajStartTime = fbk.time;
trajTime = 0;

% Execute the trajectory to the first waypoint
while (trajTime < trajectory.getDuration) && ~abortFlag

    fbk = armGroup.getNextFeedback();

    % Check for keyboard input and break out of the main loop
    % if the ESC key is pressed.  
    keys = read(kb);    
    if keys.ESC == 1
        abortFlag = true;
        break;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % If you want to do something with the latest feedback to
    % change the commands, replan a trajectory, abort, or do 
    % anything else, this is a pretty good place to do it.    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Get commanded positions, velocities, and accelerations
    % from the new trajectory state at the current time
    trajTime = fbk.time - trajStartTime;
    [pos, vel, accel] = trajectory.getState(trajTime);

    % Compensate for gravity
    gravCompEffort = armKin.getGravCompEfforts( ...
                                fbk.position, gravityVec );

    % Compensate for dynamics based on the new commands
    accelCompEffort = armKin.getDynamicCompEfforts(...
        fbk.position, ... % Used for calculating jacobian
        pos, vel, accel);

    % Send to hardware
    cmd.position = pos;
    cmd.velocity = vel;
    cmd.effort = gravCompEffort + accelCompEffort + ...
                                            effortOffset;
    armGroup.send(cmd);
end

% Hang out at the first waypoint until we press SPACE
disp('  '); 
disp('Ready to begin playback.');
disp('Press SPACE again to begin.');

while keys.SPACE == 0
    
    fbk = armGroup.getNextFeedback();
    
    cmd.position = fbk.positionCmd;
    cmd.velocity = fbk.velocityCmd;
    cmd.effort = fbk.effortCmd;
    armGroup.send(cmd);

    % Command the gripper to the most recent state
    if gripStates(1,1)==1
        gripperCmd.effort = gripperCloseEffort;
    else
        gripperCmd.effort = gripperOpenEffort;
    end
    gripperGroup.send(gripperCmd);
    
    keys = read(kb);
end

% Hang out at the first waypoint until we press SPACE
disp('Beginning playback.');
disp('Press ESC to stop.');

%%%%%%%%%%%%%%%%%%%%%%%%
% Move along waypoints %
%%%%%%%%%%%%%%%%%%%%%%%%
while ~abortFlag
    
    % Split waypoints into individual movements
    numMoves = size(waypoints,1);
    for i = 2:numMoves

        if abortFlag
            break;
        end

        fbk = armGroup.getNextFeedback();

        % Select the appropriate start and end positions
        startPosition = waypoints(i-1,:);
        endPosition = waypoints(i,:);

        moveWaypoints = [ startPosition;
                          endPosition ];

        % Make a new point-to-point trajectory and update the offset
        % for timing the trajectory.  This uses the non-blocking
        % trajectory API so that we can easily look at feedback from
        % from the arm while it is running.
        trajectory = trajGen.newJointMove( moveWaypoints );
        trajStartTime = fbk.time;
        trajTime = 0;

        % Update the gripper command for this waypoint, after reaching
        % the waypoint.  The actual gripper command gets set in the loop
        % below that executes the trajectory.
        if gripStates(i,1)==1
            gripperCmd.effort = gripperCloseEffort;
        else
            gripperCmd.effort = gripperOpenEffort;
        end

        while (trajTime < trajectory.getDuration) && ~abortFlag

            fbk = armGroup.getNextFeedback();

            % Check for keyboard input and break out of the main loop
            % if the ESC key is pressed.  
            keys = read(kb);    
            if keys.ESC == 1
                abortFlag = true;
                break;
            end

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % If you want to do something with the latest feedback to
            % change the commands, replan a trajectory, abort, or do 
            % anything else, this is a pretty good place to do it.    
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%              

            % Get commanded positions, velocities, and accelerations
            % from the new trajectory state at the current time
            trajTime = fbk.time - trajStartTime;
            [pos, vel, accel] = trajectory.getState(trajTime);

            % Compensate for gravity
            gravCompEffort = armKin.getGravCompEfforts( ...
                                        fbk.position, gravityVec );

            % Compensate for dynamics based on the new commands
            accelCompEffort = armKin.getDynamicCompEfforts(...
                fbk.position, ... % Used for calculating jacobian
                pos, vel, accel);

            % Send to hardware
            cmd.position = pos;
            cmd.velocity = vel;
            cmd.effort = gravCompEffort + accelCompEffort + ...
                                                    effortOffset;
            armGroup.send(cmd);
            gripperGroup.send(gripperCmd);
        end
        
    end
        
    % Break main loop if we're here because we aborted
    if abortFlag
        break;
    end
    
    % Otherwise go back to home position and repeat.
    % This uses the blocking API, which means you can't easily look at 
    % feedback while it is exectuting.
    startPosition = waypoints(end,:);
    endPosition = waypoints(1,:);
    
    movePositions = [ startPosition;
                      endPosition ];
              
    trajectory = trajGen.newJointMove( movePositions );
    trajGen.executeTrajectory( armGroup, trajectory, ...
                              'EnableDynamicsComp', true, ...
                              'GravityVec', gravityVec, ...
                              'EffortOffset', effortOffset);
                          
    % Command the gripper back to the initial state
    if gripStates(1,1)==1
        gripperCmd.effort = gripperCloseEffort;
    else
        gripperCmd.effort = gripperOpenEffort;
    end
    gripperGroup.send(gripperCmd);

end

disp('  ');
disp('Quitting playback.');
disp('Plotting logged feedback.');


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Stop background logging and visualize %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if enableLogging
    
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

