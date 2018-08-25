%% Setup
clear *;
close all;

% Robot specific setup. Edit as needed.
[group, kin, gravityVec] = setupArm_elbowScanner();

% Load waypoint data
load('pipeData_default.mat');

originAdjustXYZ = [  0.000;
                     0.000;
                     0.000 ]; % m

elbowOriginXYZ = elbowOriginXYZ + originAdjustXYZ';    

surfaceRasterSpacing_mm = 20;  % mm

pipeCircumference = pi * meanPipeDiameter;
surfaceRasterSpacing = surfaceRasterSpacing_mm / 1000; % convert to m

% Pipe Parameters, Downsampling
elbowSweepRes = 15;
pipeSurfaceRes = round( pipeCircumference / surfaceRasterSpacing );

meanPipeDiameter = 1.00 * meanPipeDiameter;
elbowSweepAngle = deg2rad( 90 );

[pipeCenters, pipeSurfacePoints, elbowSweepAngles] =  getPipeModelPoints( ...
    elbowBendRadius, meanPipeDiameter, elbowSweepAngle, elbowOriginXYZ, ...
    elbowSweepRes, pipeSurfaceRes );

% Trajectory
trajGen = HebiTrajectoryGenerator(kin);

% Keyboard input
kb = HebiKeyboard();

% Display Kinematic Frames
frameDisp = FrameDisplay();

% Command Struct
cmd = CommandStruct();

% Select whether you want to log and visualize the replay movement
enableLogging = true;
logDirectory = 'logs';

% Choose if we want to preview the IK solutions
visualizeIK = false;

%% Record starting IK configuration in gravity compensated mode
disp('Select starting IK configurations with ALT.');
disp('  Choose positions spaced evenly around the pipe.');
disp('  Skip this step by pressing ESC.  This will use default congifurations.');

waypoints = [];
keys = read(kb);
prevKeys = keys;

while keys.ESC == 0
    
    keys = read(kb);
    
    % Do grav-comp while training waypoints
    fbk = group.getNextFeedback();
    cmd.effort = kin.getGravCompEfforts(fbk.position, gravityVec);
    group.send(cmd);
    
    % Add new waypoint on space bar press
    keys = read(kb);
    if keys.ALT == 1 && prevKeys.ALT == 0 % diff state
        
        waypoints(end+1,:) = fbk.position;
        disp('Choose next configuration with ALT.');
        
    end
    prevKeys = keys;
    
end

if isempty(waypoints)
    % If skipped training, load saved 
    load('defaultIKPositions');
    disp('Loading previously saved IK seed positions.');
    demonstratedIKRasters = demonstratedIKRasters /...
                max(demonstratedIKRasters) * pipeSurfaceRes;
    demonstratedIKRasters(1) = 1;
else
    demonstratedIKPositions = waypoints;
    demonstratedIKRasters = linspace(1,pipeSurfaceRes,size(waypoints,1));
    save( 'latestIKPositions', ....
          'demonstratedIKPositions', 'demonstratedIKRasters');
end

waypoints = [];

initIKRasters = 1:pipeSurfaceRes;
initIKPositions = interp1( demonstratedIKRasters, ...
                           demonstratedIKPositions, ...
                           initIKRasters );

numRasters = round(1 * pipeSurfaceRes);
startRaster = 0; % Corresponds to 6 o'clock

%% Get IK for sweeping the pipe
for j=1:numRasters
    
    rasterIndex = startRaster + j;

    initPosition = initIKPositions(rasterIndex,:);
    
    for i=1:length(elbowSweepAngles)
        
        tipXYZ = squeeze(pipeSurfacePoints(:,rasterIndex,i));

        tipVector = pipeCenters(:,i) - tipXYZ;
        tipAxis(:,i) = tipVector / norm(tipVector);

        waypoints(i,:) = kin.getIK( 'xyz', tipXYZ, ... 
                                   'tipAxis', tipAxis(:,i), ...
                                   'initial', initPosition );
    end

    if rem(j,2)==0
        waypoints = flipud(waypoints);
    end
    
    rasterPoints(:,:,j) = waypoints;

    if visualizeIK
        % Plot results for sweeping the pipe
        for i=1:length(elbowSweepAngles)

            % Calculate kinematics
            frames = kin.getFK('output', waypoints(i,:));

            % Draw coordinate frames
            frameDisp.setFrames(frames);
            drawnow;

            tipAxis(:,i)
            pause(0.01);
        end
    end

end

%% Replay waypoints
disp(['Replaying ' num2str(size(waypoints,1)) ' waypoints.']);
disp('   Press ALT to begin.');

keys = read(kb);
while keys.ALT == 0
    
    keys = read(kb);
    
    % Do grav-comp while training waypoints
    fbk = group.getNextFeedback();
    cmd.effort = kin.getGravCompEfforts(fbk.position, gravityVec);
    group.send(cmd);
    
end

currentPosition = group.getNextFeedback().position;
startPosition = rasterPoints(1,:,1);

trajGen.setMinDuration(1.00); 
trajGen.setSpeedFactor(0.33); 

trajGen.moveJoint( group, [currentPosition; startPosition], ...
                    'EnableDynamicsComp', true, ...
                    'GravityVec', gravityVec );

% Start background logging 
if enableLogging
   logFile = group.startLog('dir',logDirectory); 
end          

trajGen.setMinDuration(0.50); 
trajGen.setSpeedFactor(0.66); 
                
numLoops = 1;
                
for j=1:numLoops
    for i=1:numRasters
        keys = read(kb);
        if keys.ESC == 1
            break;
        end
        
        waypoints = rasterPoints(:,:,i);

        % Move through all waypoints as a single movement.  Get the
        % auto-generated trajectory, then stretch time a little bit to
        % avoid jerkiness when stoping and starting, and then recompute.
        traj = trajGen.newJointMove( waypoints );    
        trajTime = traj.getWaypointTime();
        
        startStopTimeBuffer = .5; % seconds
        trajTime(2:end) = trajTime(2:end) + startStopTimeBuffer;
        trajTime(end) = trajTime(end) + startStopTimeBuffer;
       
        traj = trajGen.newJointMove( waypoints, 'time', trajTime );    
        
        trajGen.executeTrajectory( group, traj, ...
                            'EnableDynamicsComp', true, ...
                            'GravityVec', gravityVec );
                        
%         trajGen.moveJoint( group, waypoints, ...
%                             'EnableDynamicsComp', true, ...
%                             'GravityVec', gravityVec );
                        
        keys = read(kb);
        if keys.ESC == 1
            break;
        end
                        
        % Move to next set of raster points, if not the end
        if i < numRasters
            waypoints = [ squeeze(rasterPoints(end,:,i));
                          squeeze(rasterPoints(1,:,i+1)) ];
                      
            traj = trajGen.newJointMove( waypoints );           
            trajGen.executeTrajectory( group, traj, ...
                                'EnableDynamicsComp', true, ...
                                'GravityVec', gravityVec );

%             trajGen.moveJoint( group, waypoints, ...
%                                 'EnableDynamicsComp', true, ...
%                                 'GravityVec', gravityVec );   
        end
    end
    
    keys = read(kb);
    if keys.ESC == 1
        break;
    end
end   


%%
% Stop background logging
if enableLogging
   log = struct( group.stopLogFull() );
end

%%
% Plot Log Data
if enableLogging
   HebiUtils.plotLogs(log, 'position');
   HebiUtils.plotLogs(log, 'velocity');
   HebiUtils.plotLogs(log, 'effort');
end

%%
% Kinematics Plotting / Analysis
plotEndEffectorError( log, kin, group );
