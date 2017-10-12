%% Setup
% Robot specific setup. Edit as needed.
[group, kin, gravityVec] = setupFeedingArm();

load('defaultFeedingWaypoints');
defaultWaypoints = waypoints;

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
   plotLogCommands(hebilog, group);
end

