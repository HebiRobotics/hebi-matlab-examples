% Demo of a 4-DoF arm for scanning a flate plate
% 
% Dave Rollinson
% Jul 2018

clear *;
close all;

HebiLookup.initialize();

% Robot specific setup. Edit as needed.
kin = HebiKinematics('scanningArm_4DoF');
gains = HebiUtils.loadGains('scanningArm_4DoF_gains');
  
%%
familyName = 'scanArm';
moduleNames = {'Base','Choulder','Elbow','Wrist'};
armGroup = HebiLookup.newGroupFromNames( familyName, moduleNames );
cmd = CommandStruct();

armGroup.send('gains',gains);
armGroup.setFeedbackFrequency(200);
numDoF = armGroup.getNumModules();

fbk = armGroup.getNextFeedback();

%%
ioBoardName = '_scanIO';
ioGroup = HebiLookup.newGroupFromNames( familyName, ioBoardName );
cmdIO = IoCommandStruct();

% Pin mappings for IO Board
setX = 'e1';      % [ticks] to increment
setY = 'e3';      % [ticks] to increment
% resetXY = 'e2';    % Non-zero value resets encoder ticks to 0.
% resetXY = 'e4';    % Does same thing as 'e2'.
scannerSetX = 'e5';
scannerSetY = 'e6';
scannerClearData = 'e7';

readX = 'c1';
readY = 'c4';

% TODO: Make this a function and make the set more reliable
scanArm_IO_reset;

%%
phoneName = 'Future in My Pocket'; 
while true        
    try
        fprintf('Searching for phone Controller...\n');
        phoneGroup = HebiLookup.newGroupFromNames( familyName, phoneName );
        disp('Phone Found.  Starting up');
        break;
    catch
        pause(1.0);
    end
end

pauseFlag = false;

phoneGroup.setFeedbackFrequency(200);
phoneFbkIO = phoneGroup.getNextFeedbackIO();

% IO mapping for phone
startStop = 'b8';
resetScanner = 'b4';
handPositionMode = 'b1';
joyX = 'a1';
joyY = 'a2';
joyScale = .200;
scanSpeed = 'a3';

encoderResX = 10 * 1000; % [tics / mm] * [mm / m]
encoderResY = 10 * 1000; % [tics / mm] * [mm / m]

% Trajectory
trajGen = HebiTrajectoryGenerator(kin);
% trajGen.setMinDuration(0.1); % Min move time for 'small' movements
%                              % (default is 1.0)
% trajGen.setSpeedFactor(1.0); % Slow down movements to a safer speed.
%                              % (default is 1.0)

% Keyboard input
kb = HebiKeyboard();

% Select whether waypoints should be done as a single trajectory, or
% multiple trajectories that stop in between.
stopBetweenWaypoints = true;

%% Record waypoints in gravity compensated mode
abortFlag = false;

gravityVec = [ 0 0 -1 ];
kin.setPayload( 0.1 );  % kg


% Raster Params
rasterLimitsXY_mm = [70 70];
rasterWidth_mm = 1.0;
rasterSpeed_mm = 200; % [m/sec]

rasterLimitsXY = rasterLimitsXY_mm / 1000;% [m]
rasterWidth = rasterWidth_mm / 1000;% [m]
rasterSpeed = rasterSpeed_mm / 1000;% [m / sec]

waypointSpacing_mm = 5;
waypointSpacing = waypointSpacing_mm / 1000; % [m]

fbk = armGroup.getNextFeedback();
tLast = fbk.time;

cmdPosition = fbk.position';
cmdVelocity = zeros(size(cmdPosition));
cmdFK = kin.getFK('EndEffector', cmdPosition);
cmdXYZ = cmdFK(1:3,4);
    
while true
    
    fbk = armGroup.getNextFeedback();        
    dt = fbk.time - tLast;
    tLast = fbk.time;
    
    newPhoneFbkIO = phoneGroup.getNextFeedbackIO( 'timeout', 0 );
    if ~isempty(newPhoneFbkIO)
        phoneFbkIO = newPhoneFbkIO;
    end
    
    if ~exist('probeXYZ_init','var') || phoneFbkIO.(resetScanner)
        
        % Clear data
        cmdIO.(scannerClearData) = 1; % Digital Input 3 (Clear data)
        ioGroup.send(cmdIO);
        pause(0.06);

        cmdIO.(scannerClearData) = 0; % Digital Input 3 (Clear data)
        ioGroup.send(cmdIO);
        pause(0.06);
        
        probeFK = kin.getFK('EndEffector', fbk.position);
        probeXYZ_init = probeFK(1:3,4) - .5*[rasterLimitsXY'; 0];
    end
    
    % Do grav-comp while training waypoints
    cmd.effort = kin.getGravCompEfforts(fbk.position, gravityVec);
    
    if phoneFbkIO.(handPositionMode)
        cmdFK = kin.getFK('EndEffector', fbk.position);
        cmdXYZ = cmdFK(1:3,4);
        cmdPosition = kin.getIK( 'XYZ', cmdXYZ, ...
                                 'tipAxis', [0 0 -1], ...
                                 'initial', fbk.position );
        
        cmd.position = cmdPosition;
        cmd.velocity = nan(1,numDoF);
    else
        xyzVel = joyScale * [ sign(phoneFbkIO.(joyX)) * (phoneFbkIO.(joyX))^2;
                              sign(phoneFbkIO.(joyY)) * (phoneFbkIO.(joyY))^2;
                              0 ];
        cmdXYZ = cmdXYZ + xyzVel*dt;
        
        J = kin.getJacobianEndEffector(fbk.position);
        J = J(1:3,:);
        
        cmdVelocity = J \ xyzVel;           
        cmdPosition = kin.getIK( 'XYZ', cmdXYZ, ...
                                 'tipAxis', [0 0 -1], ...
                                 'initial', fbk.position );
        
        cmd.velocity = cmdVelocity';
        cmd.position = cmdPosition;
    end
    armGroup.send(cmd);
    
    
    % Get probe position from FK and update the scanner
    probeFK = kin.getFK('EndEffector', fbk.position);
    probeXYZ = probeFK(1:3,4) - probeXYZ_init;
    cmdIO.(setX) = round(encoderResX * probeXYZ(1));
    cmdIO.(setY)  = round(encoderResY * probeXYZ(2));
    ioGroup.send(cmdIO);
    
    % If we press a button on the phone:
    % Define the origin begin raster scan
    if phoneFbkIO.(startStop)
        probeFK = kin.getFK('EndEffector', fbk.position);
        probeXYZ_init = probeFK(1:3,4);
        ikSeedPos = fbk.position;
        break;
    end

end

% TODO: Make this a function and make the set more reliable
scanArm_IO_reset;

%% Build the raster plan
% xPts = (0:rasterWidth:rasterLimitsXY(1)) + probeXYZ_init(1);
% yPts = (0:waypointSpacing:rasterLimitsXY(2)) + probeXYZ_init(2);

xPts = (0:waypointSpacing:rasterLimitsXY(1)) + probeXYZ_init(1);
yPts = (0:rasterWidth:rasterLimitsXY(2)) + probeXYZ_init(2);

zPt = 0.000 + probeXYZ_init(3);
waypoints = nan(length(xPts),numDoF,length(yPts));

numRasters = length(yPts);
numWaypoints = length(xPts);

for i = 1:numRasters
    
    % Build a run of waypoints
    for j = 1:numWaypoints
        targetXYZ = [ xPts(j); 
                      yPts(i); 
                      zPt ];
        waypoints(j,:,i) = kin.getIK( 'XYZ', targetXYZ, ...
                                      'tipAxis', [0 0 -1], ...
                                      'initial', ikSeedPos );
        ikSeedPos = waypoints(j,:,i);
    end
    
    % Once we've built a run, flip the xPts so they run back and forth
    xPts = flip(xPts);
end

% Start background logging 
logFile = armGroup.startLog('dir','logs'); 


%%
disp('Rastering...')

movingBackwards = false;

% Move along the rasters

% Split waypoints into individual movements
numMoves = size(waypoints,3);
for i = 1:numMoves

    fbk = armGroup.getNextFeedback();
    
    moveWaypoints = waypoints(:,:,i);
    tMax = rasterLimitsXY(2) / rasterSpeed;
    trajTime = linspace(0,tMax,numWaypoints);

    % Stretch the first and last waypoint times to avoid jerking
    % at start/stop
    startStopTimeBuffer = 2*trajTime(2); % seconds
    trajTime(2:end) = trajTime(2:end) + startStopTimeBuffer;
    trajTime(end) = trajTime(end) + startStopTimeBuffer;

    traj = trajGen.newJointMove( moveWaypoints, 'time', trajTime );
    [cmd, cmdIO, abortFlag] = executeTrajectory( armGroup, phoneGroup, ioGroup, ...
                                    cmd, cmdIO, kin, traj, probeXYZ_init );
    
    if i < numRasters
        moveWaypoints = [ squeeze( waypoints(end,:,i) );
                          squeeze( waypoints(1,:,i+1) ) ];
    
        rasterSwitchTime = 0.1; % [sec]              
        traj = trajGen.newJointMove( moveWaypoints, ...
                                     'time', [0 rasterSwitchTime] );
        [cmd, cmdIO, abortFlag] = executeTrajectory( armGroup, phoneGroup, ioGroup, ...
                                    cmd, cmdIO, kin, traj, probeXYZ_init );
        
    end
end


%%
% Stop background logging
log = armGroup.stopLogFull();

%%
% Plotting Joint Data
HebiUtils.plotLogs(log, 'position', 'figNum', 101);
HebiUtils.plotLogs(log, 'velocity', 'figNum', 102);
HebiUtils.plotLogs(log, 'effort', 'figNum', 103);

scanningArmKinematics( log );




