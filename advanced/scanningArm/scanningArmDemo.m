% Demo of a 4-DoF arm for scanning a flate plate
% 
% Dave Rollinson
% Jul 2018

% Robot specific setup. Edit as needed.
kin = HebiKinematics('scanningArm_4DoF');
gains = HebiKinematics('scanningArm_4DoF_gains');

effortOffset = params.effortOffset;
gravityVec = params.gravityVec;

% Trajectory
trajGen = HebiTrajectoryGenerator(kin);
trajGen.setMinDuration(2.0); % Min move time for 'small' movements
                             % (default is 1.0)
trajGen.setSpeedFactor(0.5); % Slow down movements to a safer speed.
                             % (default is 1.0)
% Keyboard input
kb = HebiKeyboard();

% Select whether waypoints should be done as a single trajectory, or
% multiple trajectories that stop in between.
stopBetweenWaypoints = true;

group.startLog('dir','logs');

%% Record waypoints in gravity compensated mode
disp('Add waypoint with ALT.  Exit teaching mode with ESC');

waypoints = [];
keys = read(kb);
prevKeys = keys;
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

% Move from current position to first waypoint
startPosition = group.getNextFeedback().position;
trajGen.moveJoint( group, [startPosition; waypoints(1,:)], ...
    'EnableDynamicsComp', true, ...
    'GravityVec', gravityVec, ...
    'EffortOffset', effortOffset);

% Move along waypoints
while true
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
                'GravityVec', gravityVec, ...
                'EffortOffset', effortOffset );

        end
        
        keys = read(kb);    
        if keys.ESC == 1
            break;
        end

    else

        % Move through all waypoints as a single movement
        trajGen.moveJoint( group, waypoints, ...
            'EnableDynamicsComp', true, ...
            'GravityVec', gravityVec, ...
            'EffortOffset', effortOffset );
        
        keys = read(kb);    
        if keys.ESC == 1
            break;
        end
    end
    
    % Go home
    startPosition = waypoints(end,:);
    endPosition = waypoints(1,:);

    trajGen.moveJoint( group, [startPosition; endPosition], ...
        'EnableDynamicsComp', true, ...
        'GravityVec', gravityVec, ...
        'EffortOffset', effortOffset );
end

% Stop background logging and visualize
if enableLogging
    hebilog = group.stopLogFull();
    HebiUtils.plotLogs(hebilog, 'position', 'figNum', 101);
    HebiUtils.plotLogs(hebilog, 'velocity', 'figNum', 102);
    HebiUtils.plotLogs(hebilog, 'effort', 'figNum', 103);
end

