%% Setup
% Robot specific setup. Edit as needed.
clear *;
close all;

[armGroup, armKin, gravityVec] = setupArm_beverages();

[joy, joyNames] = setupJoystick();

gripper = HebiLookup.newGroupFromNames('beverageGripper','Spool');
gripper.setCommandLifetime(0);

ioGroup = HebiLookup.newGroupFromNames('beverageIO','moreBeerPlease');
ioFbk = ioGroup.getNextFeedbackIO();

% Trajectory
trajGen = HebiTrajectoryGenerator(armKin);
trajGen.setMinDuration(0.8); % Min move time for 'small' movements
                             % (default is 1.0)
trajGen.setSpeedFactor(0.9); % Slow down movements to a safer speed.
                             % (default is 1.0)

% Select whether waypoints should be done as a single trajectory, or
% multiple trajectories that stop in between.
stopBetweenWaypoints = true;

% Select whether you want to log and visualize the replay movement
enableLogging = false;

%% Record waypoints in gravity compensated mode
disp('Add waypoint with ALT.  Exit teaching mode with ESC');

waypoints = [];
gripState = false;

gripCloseEffort = -4; % Nm
gripOpenEffort = 1; % Nm

gripPayload = .5 * .350; % kg

buttons = button(joy);
prevButtons = buttons;

cmd = CommandStruct();
gripCmd = CommandStruct();

while buttons(joyNames.RIGHT_STICK_CLICK) == 0
    
    % Do grav-comp while training waypoints
    fbk = armGroup.getNextFeedback();
    cmd.effort = armKin.getGravCompEfforts(fbk.position, gravityVec);
    armGroup.send(cmd);
    
    if isempty(gripState) || gripState(end)==0
        gripCmd.effort = gripOpenEffort;
    else
        gripCmd.effort = gripCloseEffort;
    end
    
    gripper.send(gripCmd);
    
    % Add new waypoint on ALT press
    buttons = button(joy);
    if buttons(joyNames.SQUARE_BUTTON) == 1 && ...
            prevButtons(joyNames.SQUARE_BUTTON) == 0 % diff state
        
        waypoints(end+1,:) = fbk.position;
        gripState(end+1) = gripState(end);
        
        disp('Enter next waypoint.  Exit teaching mode with ESC');
        
        
    end
    
    % Toggle gripper on CTRL press
    if buttons(joyNames.X_BUTTON) == 1 && ...
            prevButtons(joyNames.X_BUTTON) == 0 % diff state   
        waypoints(end+1,:) = fbk.position;
        gripState(end+1) = ~gripState(end);  
        
        if gripState(end) == 1
            armKin.setPayload(gripPayload);
        else
            armKin.setPayload(0);
        end
        
        disp('Toggled gripper.  Exit teaching mode with ESC');
    end
    
    prevButtons = buttons;
    
end

if isempty(waypoints)
    load('beerWaypoints');
else
    % Remove the first dummy value of gripState
    gripState(1) = [];
end

%% Replay waypoints
disp(['Replaying ' num2str(size(waypoints,1)) ' waypoints'])

% Start background logging 
if enableLogging
   logFile = armGroup.startLog(); 
end

% Move from current position to first waypoint
startPosition = armGroup.getNextFeedback().position;
trajGen.moveJoint( armGroup, [startPosition; waypoints(1,:)], ...
                    'EnableDynamicsComp', true, ...
                    'GravityVec', gravityVec );

while true
    
    ioFbk = ioGroup.getNextFeedbackIO();
    buttonIsUp = ioFbk.b1;
    
    fbk = armGroup.getNextFeedback();
    
    buttons = button(joy);
    
    if buttons(joyNames.RIGHT_STICK_CLICK)
        break;
    end
    
    if buttonIsUp
        cmd.position = fbk.positionCmd;
        cmd.velocity = fbk.velocityCmd;
        cmd.effort = fbk.effortCmd;
        
        armGroup.send(cmd,'led',[]);
        gripper.send('led',[]);
        continue;
    end
    
    disp('Beer Me!');
    armGroup.send('led','w');
    gripper.send('led','w');
    
    % Set initial state of the gripper
    if gripState(1) == 1
        armKin.setPayload(gripPayload);
        gripCmd.effort = gripCloseEffort;
    else
        armKin.setPayload(0);
        gripCmd.effort = gripOpenEffort;
    end
    gripper.send(gripCmd);
        
    % Split waypoints into individual movements
    numMoves = size(waypoints,1);
    for i = 2:numMoves

        % Pick start and end positions
        startPosition = waypoints(i-1,:);
        endPosition = waypoints(i,:);
        
        if gripState(i) == 1
            gripCmd.effort = gripCloseEffort;
        else
            gripCmd.effort = gripOpenEffort;
        end
        gripper.send(gripCmd);
        
        % Do minimum-jerk trajectory between positions. Note that this
        % call handles trajectory commands internally and blocks until
        % the move is finished.
        trajGen.moveJoint( armGroup, [startPosition; endPosition], ...
                            'EnableDynamicsComp', true, ...
                            'GravityVec', gravityVec );
           
        % Set the payload after the move to avoid jerking before changing
        % the grip
        if gripState(i) == 1
            armKin.setPayload(gripPayload);
        else
            armKin.setPayload(0);
        end               
        
        buttons = button(joy);
        if buttons(joyNames.RIGHT_STICK_CLICK)
            break;
        end                
    end

    buttons = button(joy);
    if buttons(joyNames.RIGHT_STICK_CLICK)
        break;
    end
    
    if gripState(end) == 1
        gripCmd.effort = gripCloseEffort;
    else
        gripCmd.effort = gripOpenEffort;
    end
    gripper.send(gripCmd);
    
    trajGen.moveJoint( armGroup, [waypoints(end,:);  waypoints(1,:)], ...
                                'EnableDynamicsComp', true, ...
                                'GravityVec', gravityVec );
    
    if gripState(end) == 1
        armKin.setPayload(gripPayload);
    else
        armKin.setPayload(0);
    end                        
end

% Stop background logging and visualize
if enableLogging
   hebilog = armGroup.stopLogFull();
   plotLogCommands(hebilog, armGroup);
end

