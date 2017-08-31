%% Setup
% Robot specific setup. Edit as needed.
[group, kin, gravityVec] = setupArm();

% Trajectory
trajGen = HebiTrajectoryGenerator(kin);
trajGen.setMinDuration(0.5); % Speed up 'small' movements
trajGen.setSpeedFactor(0.5); % Slow down movements to a safer speed

% Keyboard input
kb = HebiKeyboard();

% Select whether waypoints should be done as a single trajectory, or
% multiple trajectories that stop in between.
stopBetweenWaypoints = true;

% Select whether you want to log and visualize the replay movement
enableLogging = true;

%% Record waypoints in gravity compensated mode
disp('Add waypoint with ALT.  Exit teaching mode with ESC');

waypoints = [];
keys = read(kb);
prevKeys = keys;
cmd = CommandStruct();

while keys.ESC == 0
    
    % Do grav-comp while training waypoints
    fbk = group.getNextFeedback();
    cmd.effort = kin.getGravCompEfforts(fbk.position, gravityVec);
    group.send(cmd);
    
    % Add new waypoint on space bar press
    keys = read(kb);
    if keys.ALT == 1 && prevKeys.ALT == 0 % diff state
        
        waypoints(end+1,:) = fbk.position;
        disp('Enter next waypoint with ALT.  Exit teaching mode with ESC');
        
    end
    prevKeys = keys;
    
end

%% Replay waypoints
disp(['Replaying ' num2str(size(waypoints,1)) ' waypoints'])

% Start background logging 
if enableLogging
   logFile = group.startLog(); 
end

% Move along waypoints
if stopBetweenWaypoints
    
    % Move from current position to first waypoint
    startPosition = group.getNextFeedback().position;
    trajGen.moveJoint(group, [startPosition; waypoints(1,:)], ...
        'EnableDynamicsComp', true, ...
        'GravityVec', gravityVec);
    
    % Split waypoints into individual movements
    numMoves = size(waypoints,1);
    for i = 2:numMoves
        
        % Pick start and end positions
        startPosition = waypoints(i-1,:);
        endPosition = waypoints(i,:);
        
        % Do minimum-jerk trajectory between positions. Note that this
        % call handles trajectory commands internally and blocks until
        % the move is finished.
        trajGen.moveJoint(group, [startPosition; endPosition], ...
            'EnableDynamicsComp', true, ...
            'GravityVec', gravityVec);
        
    end
    
else
    
    % Move through all waypoints as a single movement
    startPosition = group.getNextFeedback().position;
    trajGen.moveJoint(group, [startPosition; waypoints], ...
        'EnableDynamicsComp', true, ...
        'GravityVec', gravityVec);
    
end

% Stop background logging and visualize
if enableLogging
   hebilog = group.stopLogFull();
   plotLogCommands(hebilog, group);
end

