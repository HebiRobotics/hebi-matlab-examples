%% Setup

% Reset the workspace
clear *;
close all;

% Robot specific setup. Edit as needed.
[ group, kin, params ] = setupArm('6dof_w_gripper');

effortOffset = params.effortOffset;
gravityVec = params.gravityVec;

% Trajectory
trajGen = HebiTrajectoryGenerator(kin);
trajGen.setMinDuration(1.0); % Min move time for 'small' movements
                             % (default is 1.0)
trajGen.setSpeedFactor(0.75); % Slow down movements to a safer speed.
                             % (default is 1.0)
% Keyboard input
kb = HebiKeyboard();

% Select whether waypoints should be done as a single trajectory, or
% multiple trajectories that stop in between.
stopBetweenWaypoints = false;

% Select whether you want to log and visualize the replay movement
enableLogging = true;

%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Record waypoints in gravity compensated mode %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('  Add waypoint with ALT.  Exit teaching mode with ESC');

waypoints = [];
keys = read(kb);
prevKeys = keys;
abortFlag = false;

cmd = CommandStruct();

while keys.ESC == 0
    
    % Do grav-comp while training waypoints
    fbk = group.getNextFeedback();
    cmd.effort = kin.getGravCompEfforts(fbk.position, gravityVec) ...
        + effortOffset;
    group.send(cmd);
    
    % Add new waypoint on space bar press
    keys = read(kb);
    if keys.ALT == 1 && prevKeys.ALT == 0 % diff state
        
        waypoints(end+1,:) = fbk.position;
        disp('  Enter next waypoint with ALT.  Exit teaching mode with ESC');
        
    end
    prevKeys = keys;
    
end

tic;
while toc < 1.0
    fbk = group.getNextFeedback();
    keys = read(kb);
end

%% 
%%%%%%%%%%%%%%%%%%%%
% Replay waypoints %
%%%%%%%%%%%%%%%%%%%%
disp(['  Replaying ' num2str(size(waypoints,1)) ' waypoints'])
disp('     Stop replay by pressing ESC.')

% Start background logging 
if enableLogging
   logFile = group.startLog(); 
end

% Move from current position to first waypoint
% This uses the blocking API, which means you can't easily look at 
% feedback while it is exectuting.
startPosition = group.getNextFeedback().position;
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

while (trajTime < trajectory.getDuration) && ~abortFlag

    fbk = group.getNextFeedback();

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
    gravCompEffort = kin.getGravCompEfforts( ...
                                fbk.position, gravityVec );

    % Compensate for dynamics based on the new commands
    accelCompEffort = kin.getDynamicCompEfforts(...
        fbk.position, ... % Used for calculating jacobian
        pos, vel, accel);

    % Send to hardware
    cmd.position = pos;
    cmd.velocity = vel;
    cmd.effort = gravCompEffort + accelCompEffort + ...
                                            effortOffset;
    group.send(cmd);
end

% Move along waypoints
while true && ~abortFlag
    
    if stopBetweenWaypoints

        % Split waypoints into individual movements
        numMoves = size(waypoints,1);
        for i = 2:numMoves
            
            if abortFlag
                break;
            end
            
            fbk = group.getNextFeedback();

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
                
                fbk = group.getNextFeedback();
                
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
                gravCompEffort = kin.getGravCompEfforts( ...
                                            fbk.position, gravityVec );

                % Compensate for dynamics based on the new commands
                accelCompEffort = kin.getDynamicCompEfforts(...
                    fbk.position, ... % Used for calculating jacobian
                    pos, vel, accel);

                % Send to hardware
                cmd.position = pos;
                cmd.velocity = vel;
                cmd.effort = gravCompEffort + accelCompEffort + ...
                                                        effortOffset;
                group.send(cmd);
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
        fbk = group.getNextFeedback();
        
        trajectory = trajGen.newJointMove( waypoints );
        trajStartTime = fbk.time;
        trajTime = 0;
            
        while (trajTime < trajectory.getDuration) && ~abortFlag
                
            fbk = group.getNextFeedback();

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
            gravCompEffort = kin.getGravCompEfforts( ...
                                        fbk.position, gravityVec );

            % Compensate for dynamics based on the new commands
            accelCompEffort = kin.getDynamicCompEfforts(...
                fbk.position, ... % Used for calculating jacobian
                pos, vel, accel);

            % Send to hardware
            cmd.position = pos;
            cmd.velocity = vel;
            cmd.effort = gravCompEffort + accelCompEffort + ...
                                                    effortOffset;
            group.send(cmd);
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
    trajGen.executeTrajectory( group, trajectory, ...
                              'EnableDynamicsComp', true, ...
                              'GravityVec', gravityVec, ...
                              'EffortOffset', effortOffset);
end


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Stop background logging and visualize %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if enableLogging
    
   hebilog = group.stopLogFull();
   
   HebiUtils.plotLogs(hebilog, 'position');
   HebiUtils.plotLogs(hebilog, 'velocity');
   HebiUtils.plotLogs(hebilog, 'effort');
   
   % Feel free to put more plotting code here
end

