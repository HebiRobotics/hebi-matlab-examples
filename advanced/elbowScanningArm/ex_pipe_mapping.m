%% Setup
clear *;
close all;

% Robot specific setup. Edit as needed.
[group, kin, gravityVec] = setupArm_elbowScanner();

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

%% Record waypoints in gravity compensated mode

waypointRate = .1; % sec

disp('Recording waypoints...');
disp('   Exit recording mode with ESC');

numClusters = 0;
keys = read(kb);
prevKeys = keys;
cmd = CommandStruct();

while keys.ESC == 0
    
    % Do grav-comp while training waypoints
    fbk = group.getNextFeedback();
    cmd.effort = kin.getGravCompEfforts(fbk.position, gravityVec);
    group.send(cmd);
    
    % Add new cluster of waypoints on key press
    keys = read(kb);
    if keys.ALT == 1 && prevKeys.ALT == 0 % diff state
        
        if numClusters==0
            group.startLog();
        else
            allWaypoints{numClusters} = waypoints;
        end
        
        numClusters = numClusters+1;
        waypoints = [];
        
        disp('Starting waypoint logging...');
        pause(.1);
        waypointTimer = tic();
    end
    prevKeys = keys;
    
    if exist('waypointTimer','var') && toc(waypointTimer) > waypointRate
        waypoints(end+1,:) = fbk.position;
        disp('New waypoint recorded.');     
        waypointTimer = tic();
    end
    
end

allWaypoints{numClusters} = waypoints;

%% Shut Down The Arm, Stop Logging
disp('Stopping Gravity Compensation.');
cmd = CommandStruct();
group.send(cmd);
log = struct( group.stopLogFull() );


%% 
% Kinematics
legLength = length(log.time);
xyzCmd = nan(legLength,3);
xyzFbk = nan(legLength,3);

% Waypoints
for j=1:numClusters
    waypoints = allWaypoints{j};
    numWaypoints = size(waypoints,1);
    xyzWaypoint{j} = nan(numWaypoints,3);
    zVecWaypoint{j} = nan(numWaypoints,3);
    
    for i=1:numWaypoints
        tipFrameWaypoint = kin.getFK('endEffector',waypoints(i,:));
        xyzWaypoint{j}(i,:) = tipFrameWaypoint(1:3,4);
        zVecWaypoint{j}(i,:) = tipFrameWaypoint(1:3,3);
    end
    
    % Calculate center of pipe elbow
    pointsA = xyzWaypoint{j};
    pointsB = xyzWaypoint{j} - zVecWaypoint{j};

    [centerPoints(j,:), distances, points] = ...
                            lineIntersect3D( pointsA, pointsB );
                        
     pipeRadius = sqrt( sum((xyzWaypoint{j} - ...
                                 centerPoints(j,:)).^2,2) );
     pipeRadii(j) = mean( pipeRadius );                 
end

elbowOriginXYZ = centerPoints(1,:);
elbowBendRadius = pipeRadii(1);
centerPoints(1,:) = [];
pipeRadii(1) = [];

pipeDiameters = 2 * pipeRadii;

meanPipeDiameter = mean(pipeDiameters);
stdPipeDiameter = std(pipeDiameters);

elbowBendRadius = elbowBendRadius - meanPipeDiameter/2;
elbowSweepAngle = deg2rad( 90 );

% The elbow origin tends to be off in the y-direction, so shift it so that
% it matches the average y-value of the centerpoints.
meanCenterPoints = mean(centerPoints);
elbowOriginXYZ(2) = meanCenterPoints(2);

% Logged feedback
for i=1:legLength   
    tipFrameFbk = kin.getFK('endEffector',log.position(i,:));
    tipFrameCmd = kin.getFK('endEffector',log.positionCmd(i,:));
    
    xyzFbk(i,:) = tipFrameFbk(1:3,4);
    xyzCmd(i,:) = tipFrameCmd(1:3,4);
end

%%
% Pipe Parameters
[pipeCenters, pipeSurfacePoints, elbowSweepAngles] = ...
                          getPipeModelPoints( elbowBendRadius, ...
                                              meanPipeDiameter, ...
                                              elbowSweepAnlge, ...
                                              elbowOriginXYZ );

%% Plotting
xyzFbk_mm = xyzFbk * 1000;
xyzCmd_mm = xyzCmd * 1000;
xyzWaypoint_mm = xyzWaypoint{j} * 1000;
centerPoint_mm = centerPoints * 1000;
points_mm = points * 1000;
pipeCenters_mm = pipeCenters * 1000;
pipeSurfacePoints_mm = pipeSurfacePoints * 1000;

xyzError_mm = xyzFbk_mm - xyzCmd_mm;

% 3D Pipe
figure(101);
plot3(xyzFbk_mm(:,1),xyzFbk_mm(:,2),xyzFbk_mm(:,3));
ax = gca;
ax.ColorOrderIndex = 1;
hold on;
% plot3(xyzWaypoint_mm(:,1),xyzWaypoint_mm(:,2),xyzWaypoint_mm(:,3),'o');
%plot3(xyzCmd_mm(:,1),xyzCmd_mm(:,2),xyzCmd_mm(:,3));
plot3(centerPoint_mm(:,1),centerPoint_mm(:,2),centerPoint_mm(:,3),'*');
% plot3(points_mm(:,1),points_mm(:,2),points_mm(:,3),'.');
plot3(pipeCenters_mm(1,:),pipeCenters_mm(2,:),pipeCenters_mm(3,:));
for i=1:size(pipeSurfacePoints,3)
    ax.ColorOrderIndex = ax.ColorOrderIndex-1;
    plot3( pipeSurfacePoints_mm(1,:,i), ...
           pipeSurfacePoints_mm(2,:,i), ...
           pipeSurfacePoints_mm(3,:,i), ':' );
end
hold off;
legend('Feedback', 'Pipe Centers', 'Fit Centerline', 'Fit Surface');
title('Pipe Visualization');
xlabel('x (mm)');
ylabel('y (mm)');
zlabel('z (mm)');
axis equal;
grid on;

save( 'pipeData_latest', ...
      'elbowBendRadius', 'meanPipeDiameter', 'elbowOriginXYZ' );

