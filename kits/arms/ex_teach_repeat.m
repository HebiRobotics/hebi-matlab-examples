 %% Setup

% Reset the workspace
clear *;
close all;

localDir = fileparts(mfilename('fullpath'));

armName = '6-DoF + gripper';
armFamily = 'Arm';

% Robot specific setup. Edit as needed.
[ armGroup, armKin, params ] = setupArm( armName, armFamily );
armGroup.setFeedbackFrequency(100);

numDoF = armKin.getNumDoF();

effortOffset = params.effortOffset;
gravityVec = params.gravityVec;

% Trajectory
trajGen = HebiTrajectoryGenerator(armKin);
trajGen.setMinDuration(2.0); % Min move time for 'small' movements
                             % (default is 1.0)
trajGen.setSpeedFactor(0.75); % Slow down movements to a safer speed.
                             % (default is 1.0)
% Keyboard input
kb = HebiKeyboard();

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
prevKeys = keys;

cmd = CommandStruct();

while keys.ESC == 0
    
    % Do grav-comp while training waypoints
    fbk = armGroup.getNextFeedback();
    cmd.effort = armKin.getGravCompEfforts(fbk.position, gravityVec) ...
                                                        + effortOffset;
    armGroup.send(cmd);
    
    % Add new waypoints 
    keys = read(kb);
    
    if keys.ALT == 1 && prevKeys.ALT == 0 % diff state     
        waypoints(end+1,:) = fbk.position;
        disp('Waypoint added.');
    end
    
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
    % If you want to do something with the lastest feedback to
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
disp('Press SPACE to begin.');

while keys.SPACE == 0
    
    fbk = armGroup.getNextFeedback();
    
    cmd.position = fbk.positionCmd;
    cmd.velocity = fbk.velocityCmd;
    cmd.effort = fbk.effortCmd;
    armGroup.send(cmd);
    
    keys = read(kb);
end

% Hang out at the first waypoint until we press SPACE
disp('Beginning playback.');
disp('Press ESC to stop.');

% Move along waypoints
while ~abortFlag
    
    if stopBetweenWaypoints

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
                % If you want to do something with the lastest feedback to
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
            
        end
        
    else

        % Move through all waypoints as a single movement.
        % This uses the non-blocking trajectory API so that we can easily
        % look at feedback from from the arm while it is running.
        if abortFlag
            break;
        end
            
        % Update feedback, mostly to get the latest timestamp
        fbk = armGroup.getNextFeedback();
        
        trajectory = trajGen.newJointMove( waypoints );
        trajStartTime = fbk.time;
        trajTime = 0;
            
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
            % If you want to do something with the lastest feedback to
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
        
        keys = read(kb);    
        if keys.ESC == 1
            abortFlag = true;
            break;
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

