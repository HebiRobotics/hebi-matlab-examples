% Arm Teach-Repeat Demo
%
% Features:      Demo with two modes.  One for moving the arm to different
%                waypoints while in a zero-force gravity-compensated mode.
%                And a second mode for playing back the waypoints.
%                WITH MOBILE I/O
%
% Requirements:  MATLAB 2013b or higher
%
% Author:        Andrew Willig
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
% Date:          Sept 2020

% Copyright 2020 HEBI Robotics

%% Setup

% Reset the workspace
clear *;
close all;

armName = 'A-2085-06';
armFamily = 'Receive';
hasGasSpring = false;  % If you attach a gas spring to the shoulder for
                       % extra payload, set this to TRUE.

[ arm, params ] = setupArm( armName, armFamily, hasGasSpring );
arm.group.setFeedbackFrequency(100);
arm.trajGen.setMinDuration(1.0);   % Min move time for 'small' movements
                                   % (default is 1.0)
arm.trajGen.setSpeedFactor(0.75);  % Slow down movements to a safer speed.
                                   % (default is 1.0)
arm.plugins = {
    HebiArmPlugins.EffortOffset(params.effortOffset)
};

localDir = params.localDir;

% === Demo configuration ===
% Select whether waypoints should be done as a single trajectory, or
% multiple trajectories that stop in between.
stopBetweenWaypoints = true;

% Select whether you want to log and visualize the replay movement
enableLogging = true;

%%
%%%%%%%%%%%%%%%%%%%%%%%
% Mobile Device Setup %
%%%%%%%%%%%%%%%%%%%%%%%
phoneFamily = 'Arm';
phoneName = 'mobileIO';

altButton = 'b1';
spaceButton = 'b2';
playbackButton = 'b3';
escButton = 'b8';

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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Record waypoints in gravity compensated mode %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Move the arm to different positions to set waypoints.');
disp('  B1  - Adds a new waypoint.');  
disp('  B8  - Exits waypoint training mode.');
disp('         If no waypoints are set, default waypoints are loaded.');
disp('  ');


waypoints = [];
while ~abortFlag
    
    % Do grav-comp while training waypoints
    try
        arm.update();
        arm.send();
    catch
        disp('Could not get robot feedback!');
        break;
    end

    % Add new waypoints on ALT (B1) press
    % We get feedback from the phone into the existing structs. The
    % timeout of 0 means that the method returns immediately and won't
    % wait for feedback. If there was no feedback, the method returns
    % empty ([]), and the data in the passed in structs does not get
    % overwritten.
    % We do this because the mobile device is typically on wireless and
    % might drop out or be really delayed, in which case we would
    % rather keep running with an old data instead of waiting here for
    % new data.
    hasNewPhoneFbk = ~isempty(phoneGroup.getNextFeedback( ...
        fbkPhoneIO, fbkPhoneMobile, ... % overwrite existing structs
        'timeout', 0 )); % prevent blocking due to bad comms
    
    % Abort goal updates if the phone didn't respond
    if ~hasNewPhoneFbk
        continue;
    end
    
    if fbkPhoneIO.(altButton)    
        waypoints(end+1,:) = arm.state.fbk.position;
        disp('Waypoint added.');
    elseif fbkPhoneIO.(escButton)
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
    waypoints = tempStruct.waypoints(:,1:arm.kin.getNumDoF());
    
    disp('  '); 
    disp('No waypoints saved.  Loading default waypoints.');  
else
    waypointDir = [localDir '/waypoints'];
    waypointFileName = 'latestWaypoints';
    save([waypointDir '/' waypointFileName], 'waypoints');
    
    disp( '  ' );
    disp( [ num2str(numWaypoints) ' waypoints saved.' ] );  
end
disp( 'Press B2 to move to first waypoint.' );
abortFlag = false;
% Stay in grav-comp mode to prevent sudden movements from effort commands 
% turning on and off.

while ~abortFlag
    arm.update();
    arm.send();
    
    hasNewPhoneFbk = ~isempty(phoneGroup.getNextFeedback( ...
        fbkPhoneIO, fbkPhoneMobile, ... % overwrite existing structs
        'timeout', 0 )); % prevent blocking due to bad comms
    
    % Abort goal updates if the phone didn't respond
    if ~hasNewPhoneFbk
        continue;
    end
    
    if fbkPhoneIO.(spaceButton) 
        abortFlag = true;
    end
    
end

%% 
%%%%%%%%%%%%%%%%%%%%
% Replay waypoints %
%%%%%%%%%%%%%%%%%%%%

% Start background logging 
if enableLogging
   logFile = arm.group.startLog( 'dir', [localDir '/logs'] ); 
end

% Move from current position to first waypoint
arm.update();
arm.clearGoal(); % in case we re-run this section
arm.setGoal(waypoints(1,:));
abortFlag = false;
while ~arm.isAtGoal() && ~abortFlag
   arm.update();
   arm.send();
   
   % Check for input and break out of the main loop
   % if the ESC key (B8) is pressed.
   hasNewPhoneFbk = ~isempty(phoneGroup.getNextFeedback( ...
       fbkPhoneIO, fbkPhoneMobile, ... % overwrite existing structs
       'timeout', 0 )); % prevent blocking due to bad comms
   
   % Abort goal updates if the phone didn't respond
   if ~hasNewPhoneFbk
       continue;
   end
   
   if fbkPhoneIO.(escButton)
       abortFlag = true;
   end
   
end

% Hang out at the first waypoint until we press B3
disp('  '); 
disp('Ready to begin playback.');
disp('Press B3 to begin.');
abortFlag = false;

while ~abortFlag
    arm.update();
    arm.send();
    
    hasNewPhoneFbk = ~isempty(phoneGroup.getNextFeedback( ...
        fbkPhoneIO, fbkPhoneMobile, ... % overwrite existing structs
        'timeout', 0 )); % prevent blocking due to bad comms
    
    % Abort goal updates if the phone didn't respond
    if ~hasNewPhoneFbk
        continue;
    end
    
    if fbkPhoneIO.(playbackButton)
        abortFlag = true;
    end
    
end

disp('Beginning playback.');
disp('Press B8 to stop.');
abortFlag = false;

% Move along waypoints in a loop
while ~abortFlag
    
    if ~stopBetweenWaypoints
        % Moves through all waypoints in one movement
        arm.setGoal(waypoints);
    else
        % Adds zero velocity/acceleration constraints at each
        % waypoint to force a short stop
        arm.setGoal(waypoints, ...
            'velocities', 0 * waypoints, ...
            'accelerations', 0 * waypoints);
    end
    
    % Execute motion
    while ~arm.isAtGoal() && ~abortFlag
       arm.update();
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       % If you want to do something with the latest feedback to
       % change the commands, replan a trajectory, abort, or do
       % anything else, this is a pretty good place to do it.
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        arm.send();
        
        hasNewPhoneFbk = ~isempty(phoneGroup.getNextFeedback( ...
            fbkPhoneIO, fbkPhoneMobile, ... % overwrite existing structs
            'timeout', 0 )); % prevent blocking due to bad comms
        
        % Abort goal updates if the phone didn't respond
        if ~hasNewPhoneFbk
            continue;
        end
        
        if fbkPhoneIO.(escButton)
            abortFlag = true;
        end
    end
    
end

disp('  ');
disp('Quitting playback.');

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Stop background logging and visualize %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if enableLogging
    
   disp('Plotting logged feedback.');
   hebilog = arm.group.stopLogFull();
   
   % Plot tracking / error from the joints in the arm
   HebiUtils.plotLogs(hebilog, 'position');
   HebiUtils.plotLogs(hebilog, 'velocity');
   HebiUtils.plotLogs(hebilog, 'effort');
   
   % Plot the end-effectory trajectory and error
   kinematics_analysis( hebilog, arm.kin );
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   % Feel free to put more plotting code here %
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

disp('DONE.');

