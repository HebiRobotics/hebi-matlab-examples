% Arm Teach-Repeat Demo
%
% Features:      Demo with two modes.  One for moving the arm to different
%                waypoints while in a zero-force gravity-compensated mode.
%                And a second mode for playing back the waypoints.
%
% Requirements:  MATLAB 2013b or higher
%
% Author:        Dave Rollinson
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
% Date:          Oct 2018

% Copyright 2017-2018 HEBI Robotics

%% Setup

% Reset the workspace
clear *;
close all;

HebiLookup.initialize();

%%
%%%%%%%%%%%%%%%%%%%%%%%
% Mobile Device Setup %
%%%%%%%%%%%%%%%%%%%%%%%

phoneFamily = 'HEBI';
phoneName = 'Mobile IO';

B1_Button = 'b1';
B8_Button = 'b8';
trajectorySpeedSlider = 'a3';

abortFlag = false;

while true  
    try
        fprintf('Searching for phone Controller...\n');
        phoneGroup = HebiLookup.newGroupFromNames( ...
                        phoneFamily, phoneName );        
        disp('Phone Found.  Starting up');
        break;
    catch
        pause(1.0);
    end

end

%%
%%%%%%%%%%%%%
% Arm Setup %
%%%%%%%%%%%%%

armName = '6-DoF R-Arm';
armFamily = 'R-Arm';
hasGasSpring = false;  % If you attach a gas spring to the shoulder for
                       % extra payload, set this to TRUE.

[ armGroup, armKin, armParams ] = setupArm( armName, armFamily, hasGasSpring );
armGroup.setFeedbackFrequency(100);

numDoF = armKin.getNumDoF();

effortOffset = armParams.effortOffset;
gravityVec = armParams.gravityVec;
localDir = armParams.localDir;

% Trajectory
trajGen = HebiTrajectoryGenerator(armKin);
trajGen.setMinDuration(1.0); % Min move time for 'small' movements
                             % (default is 1.0)
trajGen.setSpeedFactor(0.75); % Slow down movements to a safer speed.
                             % (default is 1.0)
% Keyboard input
kb = HebiKeyboard();

% Select whether waypoints should be done as a single trajectory, or
% multiple trajectories that stop in between.
stopBetweenWaypoints = true;

% Select whether you want to log and visualize the replay movement
enableLogging = true;

%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Record waypoints in gravity compensated mode %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Move the arm to different positions to set waypoints.');
disp('  B1  - Adds a new waypoint.');  
disp('  B8  - Exits waypoint training mode.');
disp('         If no waypoints are set, default waypoints are loaded.');
disp('  ');


waypoints = [];
fbkPhoneIO = phoneGroup.getNextFeedback('view','io');
prevB1 = fbkPhoneIO.(B1_Button);

cmd = CommandStruct();

while ~abortFlag
    
    % Get latest feedback
    fbk = armGroup.getNextFeedback();
    fbkPhoneMobile = phoneGroup.getNextFeedback('view','mobile');
    fbkPhoneIO = phoneGroup.getNextFeedback('view','io');
    
    % Do grav-comp while training waypoints
    cmd.effort = armKin.getGravCompEfforts(fbk.position, gravityVec) ...
                                                        + effortOffset;
    armGroup.send(cmd);
    
    % Add new waypoints 
    if fbkPhoneIO.(B1_Button) && prevB1 == 0 % diff state     
        waypoints(end+1,:) = fbk.position;
        disp('Waypoint added.');
    end
    
    prevB1 = fbkPhoneIO.(B1_Button);
    
    if fbkPhoneIO.(B8_Button)
        abortFlag = true;
    end
        
    
end

numWaypoints = size(waypoints,1);

if numWaypoints == 0
    % Load the set of default waypoints for a 6-DoF arm and trim them down 
    % to the proper number of DoF.
    waypointDir = [localDir '/waypoints/'];
    waypointFileName = 'defaultWaypoints';
    
    tempStruct = load( [waypointDir waypointFileName] );
    waypoints = tempStruct.waypoints(:,1:numDoF);
    
    disp('  '); 
    disp('No waypoints saved.  Loading default waypoints.');  
else
    waypointDir = [localDir '/waypoints'];
    waypointFileName = 'latestWaypoints';
    save([waypointDir '/' waypointFileName], 'waypoints');
    
    disp( '  ' );
    disp( [ num2str(numWaypoints) ' waypoints saved.' ] );  
end
disp( 'Press B1 to move to first waypoint.' );

% Stay in grav-comp mode to prevent sudden movements from effort commands 
% turning on and off.

B1_Press = false;

while ~B1_Press
    fbk = armGroup.getNextFeedback();
    fbkPhoneIO = phoneGroup.getNextFeedback('view','io');
    
    cmd.effort = armKin.getGravCompEfforts(fbk.position, gravityVec) ...
                                                        + effortOffset;
    armGroup.send(cmd);
    
    if fbkPhoneIO.(B1_Button)
        B1_Press = true;
    end
end

abortFlag = false;

%% 
%%%%%%%%%%%%%%%%%%%%
% Replay waypoints %
%%%%%%%%%%%%%%%%%%%%

% Start background logging 
if enableLogging
   logFile = armGroup.startLog( 'dir', [localDir '/logs'] ); 
end

% Move from current position to first waypoint
% This uses the blocking API, which means you can't easily look at 
% feedback while it is exectuting.
startPosition = armGroup.getNextFeedback().position;
endPosition = waypoints(1,:);
movePositions = [ startPosition;
                  endPosition ];
              
% Make a new point-to-point trajectory and update the offset
% for timing the trajectory.  This uses the non-blocking
% trajectory API so that we can easily look at feedback from
% from the arm while it is running.
trajectory = trajGen.newJointMove( movePositions );
trajStartTime = fbk.time;
trajTime = 0;

% Execute the trajectory to the first waypoint
while (trajTime < trajectory.getDuration) && ~abortFlag

    fbk = armGroup.getNextFeedback();
    fbkPhoneIO = phoneGroup.getNextFeedback('view','io');

    % Check for mobile input and break out of the main loop
    % if the B8 button is pressed.      
    if fbkPhoneIO.(B8_Button)
        abortFlag = true;
        break;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % If you want to do something with the latest feedback to
    % change the commands, replan a trajectory, abort, or do 
    % anything else, this is a pretty good place to do it.    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Get commanded positions, velocities, and accelerations
    % from the new trajectory state at the current time
    trajTime = fbk.time - trajStartTime;
    [pos, vel, accel] = trajectory.getState(trajTime);

    % Compensate for gravity
    gravCompEffort = armKin.getGravCompEfforts( ...
                                fbk.position, gravityVec );

    % Compensate for dynamics based on the new commands
    accelCompEffort = armKin.getDynamicCompEfforts(...
        fbk.position, ... % Used for calculating jacobian
        pos, vel, accel);

    % Send to hardware
    cmd.position = pos;
    cmd.velocity = vel;
    cmd.effort = gravCompEffort + accelCompEffort + ...
                                            effortOffset;
    armGroup.send(cmd);
end

% Hang out at the first waypoint until we press SPACE
disp('  '); 
disp('Ready to begin playback.');
disp('Press B1 to begin.');

B1_Press = false;

while ~B1_Press
    
    fbk = armGroup.getNextFeedback();
    fbkPhoneIO = phoneGroup.getNextFeedback('view','io');
    
    cmd.position = fbk.positionCmd;
    cmd.velocity = fbk.velocityCmd;
    cmd.effort = fbk.effortCmd;
    armGroup.send(cmd);
    
    if fbkPhoneIO.(B1_Button)
        B1_Press = true;
    end
end

% Hang out at the first waypoint until we press SPACE
disp('Beginning playback.');
disp('Press B8 to stop.');

abortFlag = false;

% Move along waypoints
while ~abortFlag
    
    if stopBetweenWaypoints

        % Split waypoints into individual movements
        numMoves = size(waypoints,1);
        for i = 2:numMoves
            
            if abortFlag
                break;
            end
            
            fbk = armGroup.getNextFeedback();
            fbkPhoneIO = phoneGroup.getNextFeedback('view','io');

            % Select the appropriate start and end positions
            startPosition = waypoints(i-1,:);
            endPosition = waypoints(i,:);
            
            moveWaypoints = [ startPosition;
                              endPosition ];
            
            % Make a new point-to-point trajectory and update the offset
            % for timing the trajectory.  This uses the non-blocking
            % trajectory API so that we can easily look at feedback from
            % from the arm while it is running.
            trajectory = trajGen.newJointMove( moveWaypoints );
            trajStartTime = fbk.time;
            trajTime = 0;
            
            while (trajTime < trajectory.getDuration) && ~abortFlag
                
                fbk = armGroup.getNextFeedback();
                fbkPhoneIO = phoneGroup.getNextFeedback('view','io');
                
                % Check for mobile input and break out of the main loop
                % if the B8 key is pressed.      
                if fbkPhoneIO.(B8_Button)
                    abortFlag = true;
                    break;
                end
                             
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % If you want to do something with the latest feedback to
                % change the commands, replan a trajectory, abort, or do 
                % anything else, this is a pretty good place to do it.    
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%              
                
                % Get commanded positions, velocities, and accelerations
                % from the new trajectory state at the current time
                trajTime = fbk.time - trajStartTime;
                [pos, vel, accel] = trajectory.getState(trajTime);

                % Compensate for gravity
                gravCompEffort = armKin.getGravCompEfforts( ...
                                            fbk.position, gravityVec );

                % Compensate for dynamics based on the new commands
                accelCompEffort = armKin.getDynamicCompEfforts(...
                    fbk.position, ... % Used for calculating jacobian
                    pos, vel, accel);

                % Send to hardware
                cmd.position = pos;
                cmd.velocity = vel;
                cmd.effort = gravCompEffort + accelCompEffort + ...
                                                        effortOffset;
                armGroup.send(cmd);
            end
            
        end
        
    else

        % Move through all waypoints as a single movement.
        % This uses the non-blocking trajectory API so that we can easily
        % look at feedback from from the arm while it is running.
        if abortFlag
            break;
        end
            
        % Update feedback, mostly to get the latest timestamp
        fbk = armGroup.getNextFeedback();
        fbkPhoneIO = phoneGroup.getNextFeedback('view','io');
        
        trajectory = trajGen.newJointMove( waypoints );
        trajStartTime = fbk.time;
        trajTime = 0;
            
        while (trajTime < trajectory.getDuration) && ~abortFlag
                
            fbk = armGroup.getNextFeedback();
            fbkPhoneIO = phoneGroup.getNextFeedback('view','io');

            % Check for mobile input and break out of the main loop
            % if the B8 key is pressed.  
            if fbkPhoneIO.(B8_Button)
                abortFlag = true;
                break;
            end


            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % If you want to do something with the lastest feedback to
            % change the commands, replan a trajectory, abort, or do 
            % anything else, this is a pretty good place to do it.    
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


            % Get commanded positions, velocities, and accelerations
            % from the new trajectory state at the current time
            trajTime = fbk.time - trajStartTime;
            [pos, vel, accel] = trajectory.getState(trajTime);

            % Compensate for gravity
            gravCompEffort = armKin.getGravCompEfforts( ...
                                        fbk.position, gravityVec );

            % Compensate for dynamics based on the new commands
            accelCompEffort = armKin.getDynamicCompEfforts(...
                fbk.position, ... % Used for calculating jacobian
                pos, vel, accel);

            % Send to hardware
            cmd.position = pos;
            cmd.velocity = vel;
            cmd.effort = gravCompEffort + accelCompEffort + ...
                                                    effortOffset;
            armGroup.send(cmd);
        end
        
        if fbkPhoneIO.(B8_Button)
            abortFlag = true;
            break;
        end
    end
    
    % Break main loop if we're here because we aborted
    if abortFlag
        break;
    end
    
    % Otherwise go back to home position and repeat.
    % This uses the blocking API, which means you can't easily look at 
    % feedback while it is exectuting.
    startPosition = waypoints(end,:);
    endPosition = waypoints(1,:);
    
    movePositions = [ startPosition;
                      endPosition ];
              
    trajectory = trajGen.newJointMove( movePositions );
    trajGen.executeTrajectory( armGroup, trajectory, ...
                              'EnableDynamicsComp', true, ...
                              'GravityVec', gravityVec, ...
                              'EffortOffset', effortOffset);
end

disp('  ');
disp('Quitting playback.');
disp('Plotting logged feedback.');


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Stop background logging and visualize %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if enableLogging
    
   hebilog = armGroup.stopLogFull();
   
   % Plot tracking / error from the joints in the arm
   HebiUtils.plotLogs(hebilog, 'position');
   HebiUtils.plotLogs(hebilog, 'velocity');
   HebiUtils.plotLogs(hebilog, 'effort');
   
   % Plot the end-effectory trajectory and error
   kinematics_analysis( hebilog, armKin );
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   % Feel free to put more plotting code here %
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

disp('DONE.');

