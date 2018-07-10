%% Setup
%
% This demo lets a user train waypoints for an   arm to play back.  The
% default configuration is a 5-DOF suitable for feeding someone with a
% fork.

% Robot specific setup. Edit as needed.
[group, kin, gravityVec] = setupFeedingArm();

load('defaultFeedingWaypoints');
defaultWaypoints = waypoints;

% Trajectory
trajGen = HebiTrajectoryGenerator(kin);
trajGen.setMinDuration(1.5); % Min move time for 'small' movements
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

%% Record waypoints in gravity compensated mode
disp('Add waypoint with ALT.  Exit teaching mode with ESC');
disp('Hitting ESC immediately will use a default set of waypoints');

waypoints = [];
keys = read(kb);
prevKeys = keys;
cmd = CommandStruct();

while keys.ESC == 0
    
    % Do grav-comp while training waypoints
    fbk = group.getNextFeedback();
    cmd.effort = kin.getGravCompEfforts(fbk.position, gravityVec);
    group.send(cmd);
    
    % Add new waypoint on alt key press
    keys = read(kb);
    if keys.ALT == 1 && prevKeys.ALT == 0 % diff state
        
        waypoints(end+1,:) = fbk.position;
        disp('Enter next waypoint with ALT.  Exit teaching mode with ESC');
        
    end
    prevKeys = keys;
    
end

if isempty(waypoints)
    waypoints = defaultWaypoints;
end
numWaypoints = size(waypoints,1);

% Start background logging 
if enableLogging
   logFile = group.startLog(); 
end

pause(0.2);

% Move from current position to first waypoint
startPosition = group.getNextFeedback().position;
trajGen.moveJoint( group, [startPosition; waypoints(1,:)], ...
                    'EnableDynamicsComp', true, ...
                    'GravityVec', gravityVec );
                
pause(0.2);
disp('Play back waypoints by hitting SPACE BAR.  Exit teaching mode with ESC.');
keys = read(kb);

while keys.ESC == 0    
    
    % Read keyboard input, hold commands from last trajectory move
    keys = read(kb);
    
    fbk = group.getNextFeedback();
    
    cmd.position = fbk.positionCmd;
    cmd.velocity = fbk.velocityCmd;
    cmd.effort = fbk.effortCmd;
    
    group.send(cmd);
    
    % Nudge waypoints based on keyboard input.
    if keys.DOWN || keys.UP || keys.RIGHT || keys.LEFT || keys.SHIFT
        
        % Figure what direction(s) to nudge
        % The axes may need to be changed depending on the orientation
        % of the arm relative to the operator and the settings of the
        % baseFrame in the kinematics object.
        nudgeXYZ = [0; 0; 0];
        if keys.DOWN && keys.SHIFT
            nudgeXYZ = nudgeXYZ + [0; 0; -.001];
        elseif keys.DOWN
            nudgeXYZ = nudgeXYZ + [-.001; 0; 0];
        end
        if keys.UP && keys.SHIFT
            nudgeXYZ = nudgeXYZ + [0; 0; .001];
        elseif keys.UP
            nudgeXYZ = nudgeXYZ + [.001; 0; 0];
        end
        if keys.RIGHT
            nudgeXYZ = nudgeXYZ + [0; -.001; 0];
        end
        if keys.LEFT
            nudgeXYZ = nudgeXYZ + [0; .001; 0];
        end
        
        % Iterate thru and nudge each waypoint
        for i=1:numWaypoints
            
            endEffFrame = kin.getFK('endeffector',waypoints(i,:));
            targetXYZ = endEffFrame(1:3,4);
            targetTipAxis = endEffFrame(1:3,3);
            
            adjustedXYZ = targetXYZ + nudgeXYZ;
            
            newPositions = kin.getIK( 'xyz', adjustedXYZ, ...
                                      'tipaxis', targetTipAxis, ...
                                      'initial', waypoints(i,:) );
                                  
            waypoints(i,:) = newPositions;                      
        end
        
        % Command the new positions
        cmd.position = waypoints(1,:);
        cmd.effort = kin.getGravCompEfforts( cmd.position, gravityVec );
        group.send(cmd);
        pause(.03);  % Slow things down a little in case a key is held
    end
    
    % Move along waypoints
    if keys.SPACE == 1 && prevKeys.SPACE == 0 % diff state
        
        %% Replay waypoints
        disp(['Replaying ' num2str(size(waypoints,1)) ' waypoints'])
    
        if stopBetweenWaypoints   
            
            % Split waypoints into individual movements
            numMoves = size(waypoints,1);
            for i = 2:numMoves     
                % Pick start and end positions
                startPosition = waypoints(i-1,:);
                endPosition = waypoints(i,:);

                % Do minimum-jerk trajectory between positions. Note that this
                % call handles trajectory commands internally and blocks until
                % the move is finished.
                trajGen.moveJoint( group, [startPosition; endPosition], ...
                                    'EnableDynamicsComp', true, ...
                                    'GravityVec', gravityVec );   
            end
        else
            % Move through all waypoints as a single movement
            trajGen.moveJoint( group, waypoints, ...
                                'EnableDynamicsComp', true, ...
                                'GravityVec', gravityVec );
        end
        
        % Move from current commanded end position to first waypoint
        startPosition = group.getNextFeedback().positionCmd;
        trajGen.moveJoint( group, [startPosition; waypoints(1,:)], ...
                            'EnableDynamicsComp', true, ...
                            'GravityVec', gravityVec );
                        
        disp('Play back waypoints by hitting SPACE BAR.  Exit teaching mode with ESC.');
        
    end
    
    
    prevKeys = keys;
    
end

% Stop background logging and visualize
if enableLogging
    hebilog = group.stopLogFull();
    HebiUtils.plotLogs(hebilog, 'position', 'figNum', 101);
    HebiUtils.plotLogs(hebilog, 'velocity', 'figNum', 102);
    HebiUtils.plotLogs(hebilog, 'effort', 'figNum', 103);
end

