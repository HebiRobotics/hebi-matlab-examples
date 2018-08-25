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
stopBetweenWaypoints = true;

% Select whether you want to log and visualize the replay movement
enableLogging = true;
logDirectory = 'logs';

%% Record waypoints in gravity compensated mode
disp('Add waypoint with ALT.');
disp('   Exit teaching mode with ESC');

waypoints = [];
keys = read(kb);
prevKeys = keys;
cmd = CommandStruct();

waypointRate = .5; % sec

while keys.ESC == 0
    
    % Do grav-comp while training waypoints
    fbk = group.getNextFeedback();
    cmd.effort = kin.getGravCompEfforts(fbk.position, gravityVec);
    group.send(cmd);
    
    % Add new waypoint on space bar press
    keys = read(kb);
    if keys.ALT == 1 && prevKeys.ALT == 0 % diff state
        
%         disp('Starting waypoint logging...');
%         waypointTimer = tic();
        waypoints(end+1,:) = fbk.position;
        disp('Enter next waypoint with ALT.');
         
    end
    prevKeys = keys;
    
%     if exist('waypointTimer','var') && toc(waypointTimer) > waypointRate
%         waypoints(end+1,:) = fbk.position;
%         disp('New waypoint recorded.');     
%         waypointTimer = tic();
%     end
    
end

%% Replay waypoints
disp(['Replaying ' num2str(size(waypoints,1)) ' waypoints.']);
disp('   Exit playback mode with ESC.');

% Start background logging 
if enableLogging
   logFile = group.startLog( 'dir', logDirectory ); 
end

% Move from current position to first waypoint
startPosition = group.getNextFeedback().position;
trajGen.moveJoint( group, [startPosition; waypoints(1,:)], ...
                    'EnableDynamicsComp', true, ...
                    'GravityVec', gravityVec );

keys = read(kb);
while true
    
    % Add new waypoint on space bar press
    keys = read(kb);
    if keys.ESC == 1
        break;
    end
    
    % Move along waypoints
    if stopBetweenWaypoints

        % Split waypoints into individual movements
        numMoves = size(waypoints,1);
        for i = 2:numMoves
            
            keys = read(kb);                
            if keys.ESC == 1
                break;
            end  

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
        
        keys = read(kb);
        if keys.ESC == 1
            break;
        end

        % Move through all waypoints as a single movement
        trajGen.moveJoint( group, waypoints, ...
                            'EnableDynamicsComp', true, ...
                            'GravityVec', gravityVec );
    end
    
    keys = read(kb);
    if keys.ESC == 1
        break;
    end
    
    trajGen.moveJoint( group, [waypoints(end,:);  waypoints(1,:)], ...
                                'EnableDynamicsComp', true, ...
                                'GravityVec', gravityVec );
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
end

%% 
% Kinematic Error
legLength = length(log.time);
xyzCmd = nan(legLength,3);
xyzFbk = nan(legLength,3);

for i=1:legLength
    
    tipFrameFbk = kin.getFK('endEffector',log.position(i,:));
    tipFrameCmd = kin.getFK('endEffector',log.positionCmd(i,:));
    
    xyzFbk(i,:) = tipFrameFbk(1:3,4);
    xyzCmd(i,:) = tipFrameCmd(1:3,4);
end

xyzFbk_mm = xyzFbk * 1000;
xyzCmd_mm = xyzCmd * 1000;

xyzError_mm = xyzFbk_mm - xyzCmd_mm;

%% Plotting

% 3D Tracking
figure(101);
plot3(xyzFbk_mm(:,1),xyzFbk_mm(:,2),xyzFbk_mm(:,3));
hold on;
plot3(xyzCmd_mm(:,1),xyzCmd_mm(:,2),xyzCmd_mm(:,3));
hold off;
legend('Feedback','Command');
title('End Effector Tracking');
xlabel('x (mm)');
ylabel('y (mm)');
zlabel('z (mm)');
axis equal;
grid on;

% End Effector Error
figure(102);
subplot(2,1,1);
plot(log.time, xyzError_mm);
legend('x','y','z');
title('Component End Effector Error');
xlabel('time (sec)');
ylabel('error (mm)');
grid on;

subplot(2,1,2);
plot(log.time, sqrt(sum(xyzError_mm.^2,2)),'k');
title('Total Effector Error');
xlabel('time (sec)');
ylabel('error (mm)');
grid on;




