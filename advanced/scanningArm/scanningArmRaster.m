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
  
familyName = 'scanArm';
moduleNames = {'Base','Choulder','Elbow','Wrist'};
group = HebiLookup.newGroupFromNames( familyName, moduleNames );
cmd = CommandStruct();

group.send('gains',gains);
group.setFeedbackFrequency(200);
numDoF = group.getNumModules();

%%
ioBoardName = 'scanIO';
ioGroup = HebiLookup.newGroupFromNames( familyName, ioBoardName );
cmdIO = IoCommandStruct();

% Pin mappings for IO Board
setX = 'e1';
setY = 'e3';
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
startStop = 'b1';
joyX = 'a1';
joyY = 'a2';

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
disp('Define origin with ALT.');

waypoints = [];
keys = read(kb);
prevKeys = keys;

abortFlag = false;

gravityVec = [ 0 0 -1 ];
kin.setPayload( 0.25 );  % kg


% Raster Params
rasterLimitsXY_mm = [75 75];
rasterWidth = 0.5;
rasterSpeed_mm = 200;

rasterLimitsXY = rasterLimitsXY_mm / 1000;% [m]
rasterWidth = rasterWidth / 1000;% [m]
rasterSpeed = rasterSpeed_mm / 1000;% [m / sec]

waypointSpacing_mm = 10;
waypointSpacing = waypointSpacing_mm / 1000; % [m]

while true
    
    newPhoneFbkIO = phoneGroup.getNextFeedbackIO( 'timeout', 0 );
    if ~isempty(newPhoneFbkIO)
        phoneFbkIO = newPhoneFbkIO;
    end
    
    % Do grav-comp while training waypoints
    fbk = group.getNextFeedback();
    cmd.effort = kin.getGravCompEfforts(fbk.position, gravityVec);
    group.send(cmd);
    
    % Define origin when we press a button on the phone
    if phoneFbkIO.(startStop)
        probeFK = kin.getFK('EndEffector', fbk.position);
        probeXYZ_init = probeFK(1:3,4);
        ikSeedPos = fbk.position;
        break;
    end

end

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
    
    % Once we've built a run, flip the yPts so they run back and forth
    xPts = flip(xPts);
end

% Start background logging 
logFile = group.startLog('dir','logs'); 


%%
disp('Rastering...')

% Move along the rasters

% Split waypoints into individual movements
numMoves = size(waypoints,3);
for i = 1:numMoves

    fbk = group.getNextFeedback();
    
    while pauseFlag
        
        fbk = group.getNextFeedback();
        
        newPhoneFbkIO = phoneGroup.getNextFeedbackIO( 'timeout', 0 );
        if ~isempty(newPhoneFbkIO)
            phoneFbkIO = newPhoneFbkIO;
        end
        
        if phoneFbkIO.(startStop)
            pauseFlag = false;
            pause(0.1);
        end 
        
        cmd.position = fbk.positionCmd;
        cmd.velocity = fbk.velocityCmd;
        cmd.effort = fbk.effortCmd;
        group.send(cmd);

    end
    
    moveWaypoints = waypoints(:,:,i);
    tMax = rasterLimitsXY(2) / rasterSpeed;
    trajTime = linspace(0,tMax,numWaypoints);

    % Stretch the first and last waypoint times to avoid jerking
    % at start/stop
    startStopTimeBuffer = 2*trajTime(2); % seconds
    trajTime(2:end) = trajTime(2:end) + startStopTimeBuffer;
    trajTime(end) = trajTime(end) + startStopTimeBuffer;

    traj = trajGen.newJointMove( moveWaypoints, 'time', trajTime );
    t0 = fbk.time;
    t = 0;

    while (t < traj.getDuration) && ~abortFlag

        fbk = group.getNextFeedback();

        newPhoneFbkIO = phoneGroup.getNextFeedbackIO( 'timeout', 0 );
        if ~isempty(newPhoneFbkIO)
            phoneFbkIO = newPhoneFbkIO;
        end
        
        if phoneFbkIO.(startStop)
            pauseFlag = true;
            disp('Pausing...');
        end

        % Get commanded positions, velocities, and accelerations
        % from the new trajectory state at the current time
        t = fbk.time - t0;
        [pos, vel, accel] = traj.getState(t);

        % Compensate for gravity
        gravCompEffort = kin.getGravCompEfforts( ...
                                    fbk.position, gravityVec );

        % Compensate for dynamics based on the new commands
        accelCompEffort = kin.getDynamicCompEfforts(...
            fbk.position, ... % Used for calculating jacobian
            pos, vel, accel);

        cmd.position = pos;
        cmd.velocity = vel;
        cmd.effort = gravCompEffort + accelCompEffort;
        group.send(cmd);

        % Get probe position from FK
        probeFK = kin.getFK('EndEffector', fbk.position);
        probeXYZ = probeFK(1:3,4) - probeXYZ_init;

        cmdIO.(setX) = round(encoderResX * probeXYZ(1));
        cmdIO.(setY) = round(encoderResY * probeXYZ(2));
        ioGroup.send(cmdIO);
    end
    
    if i < numRasters
        moveWaypoints = [ squeeze( waypoints(end,:,i) );
                          squeeze( waypoints(1,:,i+1) ) ];

    
        rasterSwitchTime = 0.1; % [sec]              
        traj = trajGen.newJointMove( moveWaypoints, ...
                                     'time', [0 rasterSwitchTime] );
        t0 = fbk.time;
        t = 0;
    
        while (t < traj.getDuration) && ~abortFlag

            fbk = group.getNextFeedback();

            % Check for keyboard input and break out of the main loop
            % if the ESC key is pressed.  
            keys = read(kb);    
            if keys.ESC == 1
                abortFlag = true;
                break;
            end

            % Get commanded positions, velocities, and accelerations
            % from the new trajectory state at the current time
            t = fbk.time - t0;
            [pos, vel, accel] = traj.getState(t);

            % Compensate for gravity
            gravCompEffort = kin.getGravCompEfforts( ...
                                        fbk.position, gravityVec );

            % Compensate for dynamics based on the new commands
            accelCompEffort = kin.getDynamicCompEfforts(...
                fbk.position, ... % Used for calculating jacobian
                pos, vel, accel);

            cmd.position = pos;
            cmd.velocity = vel;
            cmd.effort = gravCompEffort + accelCompEffort;
            group.send(cmd);

            % Get probe position from FK
            probeFK = kin.getFK('EndEffector', fbk.position);
            probeXYZ = probeFK(1:3,4) - probeXYZ_init;

            cmdIO.(setX) = round(encoderResX * probeXYZ(1));
            cmdIO.(setY) = round(encoderResY * probeXYZ(2));
            ioGroup.send(cmdIO);
        end
    end
end


%%
% Stop background logging
log = group.stopLogFull();

%%
% Plotting Joint Data
HebiUtils.plotLogs(log, 'position', 'figNum', 101);
HebiUtils.plotLogs(log, 'velocity', 'figNum', 102);
HebiUtils.plotLogs(log, 'effort', 'figNum', 103);





