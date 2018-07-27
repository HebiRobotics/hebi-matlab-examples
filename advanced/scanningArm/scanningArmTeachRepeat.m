% Demo of a 4-DoF arm for scanning a flate plate
% 
% Dave Rollinson
% Jul 2018

% Robot specific setup. Edit as needed.
kin = HebiKinematics('scanningArm_4DoF');
gains = HebiUtils.loadGains('scanningArm_4DoF_gains2');
  
familyName = 'scanArm';
moduleNames = {'Base','Shoulder','Elbow','Wrist'};
group = HebiLookup.newGroupFromNames( familyName, moduleNames );
group.setFeedbackFrequency(100);
cmd = CommandStruct();

group.send('gains',gains);
numDoF = group.getNumModules();

ioGroup = HebiLookup.newGroupFromNames( familyName, 'scanIO' );
cmdIO = IoCommandStruct();

scanArm_IO_reset;

% Pin mappings for IO Board
setX = 'e1';
setY = 'e3';
readX = 'c1';
readY = 'c4';

encoderResX = 10 * 1000; % [tics / mm] * [mm / m]
encoderResY = 10 * 1000; % [tics / mm] * [mm / m]

% Trajectory
trajGen = HebiTrajectoryGenerator(kin);
trajGen.setMinDuration(0.33); % Min move time for 'small' movements
                             % (default is 1.0)
trajGen.setSpeedFactor(0.4); % Slow down movements to a safer speed.
                             % (default is 1.0)
% Keyboard input
kb = HebiKeyboard();

% Select whether waypoints should be done as a single trajectory, or
% multiple trajectories that stop in between.
stopBetweenWaypoints = true;

%% Record waypoints in gravity compensated mode
disp('Add waypoint with ALT.  Exit teaching mode with ESC');

waypoints = [];
keys = read(kb);
prevKeys = keys;

gravityVec = [ 0 0 -1 ];
gravCompScale = 1.0; 
kin.setPayload( 0.0 );  % kg

% Assume that where the arm is now is zero
fbk = group.getNextFeedback();
probeFK = kin.getFK('EndEffector', fbk.position);
probeXYZ_init = probeFK(1:3,4);

while keys.ESC == 0
    
    % Do grav-comp while training waypoints
    fbk = group.getNextFeedback();
    cmd.effort = gravCompScale * ...
                    kin.getGravCompEfforts(fbk.position, gravityVec);
    group.send(cmd);
    
    % Get probe position from FK
    probeFK = kin.getFK('EndEffector', fbk.position);
    probeXYZ = probeFK(1:3,4) - probeXYZ_init;
    
    cmdIO.(setX) = round(encoderResX * probeXYZ(1));
    cmdIO.(setY) = round(encoderResY * probeXYZ(2));
    ioGroup.send(cmdIO);
    
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
logFile = group.startLog('dir','logs'); 

% Move from current position to first waypoint
startPosition = group.getNextFeedback().position;
trajGen.moveJoint( group, [startPosition; waypoints(1,:)], ...
                          'EnableDynamicsComp', true, ...
                          'GravityVec', gravityVec );

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
                                      'GravityVec', gravityVec );

        end
        
        keys = read(kb);    
        if keys.ESC == 1
            break;
        end

    else

        % Move through all waypoints as a single movement
        trajGen.moveJoint( group, waypoints, ...
                           'EnableDynamicsComp', true, ...
                           'GravityVec', gravityVec );
        
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
                              'GravityVec', gravityVec );
end

% Stop background logging and visualize

log = group.stopLogFull();
HebiUtils.plotLogs(log, 'position', 'figNum', 101);
HebiUtils.plotLogs(log, 'velocity', 'figNum', 102);
HebiUtils.plotLogs(log, 'effort', 'figNum', 103);


