function [] = scanningArmRaster()
    % Demo of a 4-DoF arm for scanning a flate plate
    % 
    % Dave Rollinson
    % Jul 2018

    HebiLookup.initialize();   
    enableLogging = false;

    % Robot specific setup. Edit as needed.
    kin = HebiKinematics('scanningArm_4DoF');
    gains = HebiUtils.loadGains('scanningArm_4DoF_gains');

    %%
    familyName = 'scanArm';
    moduleNames = {'Base','Choulder','Elbow','Wrist'};
    armGroup = HebiLookup.newGroupFromNames( familyName, moduleNames );
    cmd = CommandStruct();

    % Set gains, assume that if control strategy isn't 0 then the gains
    % were set correctly.  This covers the main use case that we're setting
    % after having pressed an e-stop.
    gainsAreSet = false;
    while ~gainsAreSet
        disp('Setting Gains');
        armGroup.send('gains',gains);
        pause(.05);
        fbkGains = armGroup.getGains();
        if all(fbkGains.controlStrategy>0)
            gainsAreSet = true;
        end
    end
    
    armGroup.setFeedbackFrequency(200);
    numDoF = armGroup.getNumModules();

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

    encoderResX = 10 * 1000; % [tics / mm] * [mm / m]
    encoderResY = 10 * 1000; % [tics / mm] * [mm / m]

    readX = 'c1';
    readY = 'c4';

    % TODO: Make this a function and make the set more reliable
    scanArm_IO_reset;

    %%
    phoneName = '_scanUI'; 
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

    phoneGroup.setFeedbackFrequency(200);
    phoneFbkIO = phoneGroup.getNextFeedbackIO();

    % IO mapping for phone
    startStop = 'b8';
    resetScanner = 'b4';
    handPositionMode = 'b1';
    joyX = 'a1';
    joyY = 'a2';
    joyScale = .100;
    scanSpeed = 'a3';
    downForce = 'a6';
    wristAdjust = 'a4';

    wristAdjustScale = .03;

    damperGains = [2; 2; 2; 0.1; 0.1; 0.0;]; % [N/(m/sec)] or [Nm/(rad/sec)]
    springGains = [200; 200; 20; 1; 1; 0];  % [N/m] or [Nm/rad]

    maxPushDownForce = 20;  % N
    pushDownWrench = [0; 0; -maxPushDownForce; 0; 0; 0];

    % Trajectory
    trajGen = HebiTrajectoryGenerator(kin);

    %% Record waypoints in gravity compensated mode
    gravityVec = [ 0 0 -1 ];
    kin.setPayload( 0.0 );  % kg

    % Raster Params
    rasterLimitsXY_mm = [70 150];
    rasterWidth_mm = 1.0;
    rasterSpeed_mm = 300; % [m/sec]

    rasterLimitsXY = rasterLimitsXY_mm / 1000;% [m]
    rasterWidth = rasterWidth_mm / 1000;% [m]
    rasterSpeed = rasterSpeed_mm / 1000;% [m / sec]

    waypointSpacing_mm = 10;
    waypointSpacing = waypointSpacing_mm / 1000; % [m]

    fbk = armGroup.getNextFeedback();
    tLast = fbk.time;

    cmdPosition = fbk.position';
    cmdVelocity = zeros(size(cmdPosition));
    cmdFK = kin.getFK('EndEffector', cmdPosition);
    cmdXYZ = cmdFK(1:3,4);

    while true
        
        while true

            fbk = armGroup.getNextFeedback();        
            dt = fbk.time - tLast;
            tLast = fbk.time;
            
            % Check for M-Stop and throw an error to restart if detected
            fbkIO = armGroup.getNextFeedbackIO();
            if any(fbkIO.a1==0)
                error('M-Stop Detected!');
            end

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

            % Add Gravity Compensation and Down-Force
            gravComp = kin.getGravCompEfforts(fbk.position, gravityVec);  
            
            J_armTip = kin.getJacobian('endeffector',fbk.position);
            downForceScale = .5 * (1 + phoneFbkIO.(downForce)); % scale [0-1]
            downForceEfforts = J_armTip' * downForceScale*pushDownWrench; 
            cmd.effort = gravComp + downForceEfforts';

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

            % Tweak the wrist a little bit
            wristPosTweak = phoneFbkIO.(wristAdjust) * wristAdjustScale;
            cmd.position(4) = cmd.position(4) + wristPosTweak;

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

        % Raster run along X
        xPts = (0:waypointSpacing:rasterLimitsXY(1)) + probeXYZ_init(1);
        yPts = (0:rasterWidth:rasterLimitsXY(2)) + probeXYZ_init(2);
        numRasters = length(yPts);
        numWaypoints = length(xPts);

        zPt = 0.000 + probeXYZ_init(3);
        waypoints = nan(length(xPts),numDoF,length(yPts));

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
            % yPts = flip(yPts);
        end

        % Start background logging 
        if enableLogging
            logFile = armGroup.startLog('dir','logs'); 
        end

        % Copy various import variables into a struct to pass into the function 
        % that executes the raster trajectories.
        otherInfo.probeXYZ_init = probeXYZ_init;

        % These are copied from the higher level.  Not the best practice :-(
        otherInfo.setX = setX;      % [ticks] to increment
        otherInfo.setY = setY;      % [ticks] to increment
        otherInfo.encoderResX = encoderResX; % [tics / mm] * [mm / m]
        otherInfo.encoderResY = encoderResY; % [tics / mm] * [mm / m]

        otherInfo.scanSpeed = scanSpeed;  
        otherInfo.downForce = downForce;
        otherInfo.maxPushDownForce = maxPushDownForce;
        otherInfo.startStop = startStop;
        otherInfo.wristAdjust = wristAdjust;
        otherInfo.wristAdjustScale = wristAdjustScale;

        otherInfo.damperGains = damperGains;  % [N/(m/sec)] or [Nm/(rad/sec)]
        otherInfo.springGains = springGains;  % [N/m] or [Nm/rad]


        %%
        % Move along the rasters
        fprintf('Rastering...')

        % Split waypoints into individual movements
        numMoves = size(waypoints,3);
        for i = 1:numMoves
            
            moveWaypoints = waypoints(:,:,i);
            tMax = rasterLimitsXY(1) / rasterSpeed;
            trajTime = linspace(0,tMax,numWaypoints);

            % Stretch the first and last waypoint times to avoid jerking
            % at start/stop
            startStopTimeBuffer = 2*trajTime(2); % seconds
            trajTime(2:end) = trajTime(2:end) + startStopTimeBuffer;
            trajTime(end) = trajTime(end) + startStopTimeBuffer;

            traj = trajGen.newJointMove( moveWaypoints, 'time', trajTime );
            [cmd, cmdIO, abortFlag] = executeTrajectory( armGroup, phoneGroup, ioGroup, ...
                                            cmd, cmdIO, kin, traj, otherInfo );

            if abortFlag
                fprintf('STOPPED!\n');
                pause(0.2);
                break;
            end

            % Shift to next raster
            if i < numRasters
                moveWaypoints = [ squeeze( waypoints(end,:,i) );
                                  squeeze( waypoints(1,:,i+1) ) ];

                rasterSwitchTime = 0.1; % [sec]              
                traj = trajGen.newJointMove( moveWaypoints, ...
                                             'time', [0 rasterSwitchTime] );
                [cmd, cmdIO, abortFlag] = executeTrajectory( armGroup, phoneGroup, ioGroup, ...
                                            cmd, cmdIO, kin, traj, otherInfo );
            end

            if abortFlag
                fprintf('STOPPED!\n');
                pause(0.2);
                break;
            end
        end
        
        if ~abortFlag
        	fprintf('DONE!\n');
        end
        
        % Get the current arm position so that it doesn't jerk back to the
        % start of the raster when resetting back up top.
        fbk = armGroup.getNextFeedback();
        cmdFK = kin.getFK('EndEffector', fbk.position);
        cmdXYZ = cmdFK(1:3,4);
    end

    %%
    % Stop background logging
    if enableLogging
        log = armGroup.stopLogFull();
    end

%     %%
%     % Plotting Joint Data
%     HebiUtils.plotLogs(log, 'position', 'figNum', 101);
%     HebiUtils.plotLogs(log, 'velocity', 'figNum', 102);
%     HebiUtils.plotLogs(log, 'effort', 'figNum', 103);
% 
%     scanningArmAnalysis( log );
end

