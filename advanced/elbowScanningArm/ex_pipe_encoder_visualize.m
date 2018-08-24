%% Setup
clear *;
close all;

% Robot specific setup. Edit as needed.
[group, kin, gravityVec] = setupArm_chevron();

ioGroup = HebiLookup.newGroupFromNames('IO_BASIC_C','curtis_test');

encoderTickScale = 40 * 1000; % ticks / mm

% Pin mappings for IO Board
setX = 'e1';
setY = 'e3';
readX = 'c1';
readY = 'c4';

% Trajectory
trajGen = HebiTrajectoryGenerator(kin);
trajGen.setMinDuration(0.5); % Min move time for 'small' movements
                             % (default is 1.0)
trajGen.setSpeedFactor(0.5); % Slow down movements to a safer speed.
                             % (default is 1.0)
% Keyboard input
kb = HebiKeyboard();

% Select whether waypoints should be done as a single trajectory, or
% multiple trajectories that stop in between.
stopBetweenWaypoints = false;

%% Record arm position / kinematics in gravity compensated mode
disp('Recording waypoints...');
disp('   Exit recording mode with ESC');

keys = read(kb);
prevKeys = keys;

cmd = CommandStruct();
ioCmd = IoCommandStruct();

xyzWaypoint = nan(0,3);
encoderXY = nan(0,2);

disp('Starting up ...');
disp('   Start recording mode with SPACE');

while keys.SPACE == 0
    
    keys = read(kb);
    
    % Do grav-comp while training waypoints
    fbk = group.getNextFeedback();
    cmd.effort = kin.getGravCompEfforts(fbk.position, gravityVec);
    group.send(cmd);
    
    % Update the commanded encoder position so that it initializes right
    % when we start the recording loop.
    
    % Get forward kinematics
    tipFrameWaypoint = kin.getFK('endEffector',fbk.position);
    tipXYZ = tipFrameWaypoint(1:3,4);
    
    tickSetXY = round( encoderTickScale * tipXYZ(1:2)' );
    ioCmd.(setX) = tickSetXY(1,1);
    ioCmd.(setY) = tickSetXY(1,2);
    ioGroup.send(ioCmd);
end

while keys.ESC == 0
  
    % Feedback from the arm and the IO Board used to spoof encoder commands
    fbk = group.getNextFeedback();
    ioFbk = ioGroup.getNextFeedbackIO();
    
    % Do grav-comp while training waypoints
    cmd.effort = kin.getGravCompEfforts(fbk.position, gravityVec);
    group.send(cmd)
    
    % Get forward kinematics
    tipFrameWaypoint = kin.getFK('endEffector',fbk.position);
    xyzWaypoint(end+1,:) = tipFrameWaypoint(1:3,4);
    
    % Set the scaled encoder value
    tickSetXY = round( encoderTickScale * xyzWaypoint(end,:) );
    ioCmd.(setX) = tickSetXY(1,1);
    ioCmd.(setY) = tickSetXY(1,2);
    ioGroup.send(ioCmd);
    
    % Get the feedback from the IO Board and plot
    encoderXY(end+1,:) = [ioFbk.(readX) ioFbk.(readY)];
    plot(encoderXY(:,1),encoderXY(:,2));
    drawnow;
    
    keys = read(kb);
end


