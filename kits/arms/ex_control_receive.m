% Webinar Control-Receiver Demo
%
% Features:      Demo where one arm is moved around and controlled by
%                another and effort feedback is sent back to control arm.
%
% Requirements:  MATLAB 2013b or higher
%
% Author:        Andrew Willig
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
% Date:          Sept 2020
%
% Copyright 2020 HEBI Robotics

%% Setup

% Reset the workspace
clear *;
close all;

HebiLookup.initialize();
HebiLookup.setLookupAddresses({'10.10.12.107', '10.10.10.255'});

controlFamily = 'Control';
receiveFamily = 'Receive';

% Keyboard input
kb = HebiKeyboard();
localDir = fileparts(mfilename('fullpath'));
controlParams.localDir = localDir;
receiveParams.localDir = localDir;

shoulderJointComp = 0;

% X-Series 6-DoF Arm
controlGroup = HebiLookup.newGroupFromNames(controlFamily, {
    'J1_base'
    'J2_shoulder'
    'J3_elbow'
    'J4_wrist1'
    'J5_wrist2'
    'J6_wrist3' });

receiveGroup = HebiLookup.newGroupFromNames(receiveFamily, {
    'J1_base'
    'J2_shoulder'
    'J3_elbow'
    'J4_wrist1'
    'J5_wrist2'
    'J6_wrist3' });

% Kinematic Model
controlKin = HebiKinematics([localDir '/hrdf/A-2085-06_control']);
receiveKin = HebiKinematics([localDir '/hrdf/A-2085-06_receive']);

% Load and send arm gains
controlParams.gains = HebiUtils.loadGains([localDir '/gains/A-2085-06_control']);
receiveParams.gains = HebiUtils.loadGains([localDir '/gains/A-2085-06_receive']);

% No Gripper
controlParams.hasGripper = false;
receiveParams.hasGripper = false;

% Compensation to joint efforts due to a gas spring (if present)
controlParams.effortOffset = [0 shoulderJointComp 0 0 0 0];
receiveParams.effortOffset = [0 shoulderJointComp 0 0 0 0];

% Default seed positions for doing inverse kinematics
controlParams.ikSeedPos = [0.01 1.0 2.5 1.5 -1.5 0.01];
receiveParams.ikSeedPos = [0.01 1.0 2.5 1.5 -1.5 0.01];

% Load Arm API
controlArm = HebiArm(controlGroup, controlKin);
HebiUtils.sendWithRetry(controlArm.group, 'gains', controlParams.gains);
receiveArm = HebiArm(receiveGroup, receiveKin);
HebiUtils.sendWithRetry(receiveArm.group, 'gains', receiveParams.gains);

controlArm.group.setFeedbackFrequency(100);
receiveArm.group.setFeedbackFrequency(100);
controlArm.trajGen.setMinDuration(6.00);   % Init move time
                                   % (default is 1.0)
receiveArm.trajGen.setMinDuration(6.00);   % Init move time
                                   % (default is 1.0)
controlArm.plugins = {HebiArmPlugins.EffortOffset(controlParams.effortOffset)};
receiveArm.plugins = {HebiArmPlugins.EffortOffset(receiveParams.effortOffset)};

% === Demo configuration ===
% Select whether waypoints should be done as a single trajectory, or
% multiple trajectories that stop in between.
stopBetweenWaypoints = true;

% Select whether you want to log and visualize the replay movement
enableLogging = true;

%% 

if enableLogging
        controlArm.group.startLog('dir',[controlParams.localDir '/logs']);
        receiveArm.group.startLog('dir',[receiveParams.localDir '/logs']);
end

% Main Setup
abortFlag = false;
keys = read(kb);

% Set starting positions
targetJoints = [0, pi/2, pi/2, pi/2, -pi/2, 0];
controlArm.update()
receiveArm.update()
controlArm.setGoal(targetJoints)
receiveArm.setGoal(targetJoints)

% Move to initial positions
while ~controlArm.isAtGoal() && ~receiveArm.isAtGoal()
    
    controlArm.update();
    receiveArm.update();
    controlArm.send();
    receiveArm.send();
    
end

controlArm.trajGen.setMinDuration(0.25);   % Min move time for 'small' movements
                                   % (default is 1.0)
receiveArm.trajGen.setMinDuration(0.25);   % Min move time for 'small' movements
                                   % (default is 1.0)
                                   

while ~abortFlag
    
    try
        % Gather latest feedback from the arms
        controlArm.update()
        receiveArm.update()
    catch
        continue;
    end
    
    controlArm.clearGoal();
%     receiveArm.isAtGoal = false;
    
    % Read the Control Arm feedback
    controlFbkPos = controlArm.state.fbk.position;
    controlFbkVel = controlArm.state.fbk.velocity;  
    
    % Send the Receive Arm its new commands
    receiveArm.setGoal(controlFbkPos, 'velocities', controlFbkVel);
    receiveArm.send();
    
    % Send effort feedback back to control arm
    effortDiff = receiveArm.state.cmdEffort - receiveArm.state.fbk.effort;
    % Get Grav Comp from Control Arm
    controlBaseX = controlArm.state.fbk.orientationX(1);
    controlBaseY = controlArm.state.fbk.orientationY(1);
    controlBaseZ = controlArm.state.fbk.orientationZ(1);
    gravityVec = [controlBaseX controlBaseY controlBaseZ];
    gravCompEffort = controlArm.kin.getGravCompEfforts(controlArm.state.fbk.position, gravityVec);
    scale = 0.5;
    controlArm.state.cmdEffort = gravCompEffort + (scale * effortDiff);
    controlArm.send();
    
    % Check for keyboard input and break out of the main loop
    % if the ESC key is pressed.
    keys = read(kb);
    abortFlag = keys.ESC;

end

%%
if enableLogging
    
   controlLog = controlArm.group.stopLogFull();
   receiveLog = receiveArm.group.stopLogFull();
   
   % Plot tracking / error from the joints in the arm.  Note that there
   % will not by any 'error' in tracking for position and velocity, since
   % this example only commands effort.
   HebiUtils.plotLogs(controlLog, 'position');
   HebiUtils.plotLogs(controlLog, 'velocity');
   HebiUtils.plotLogs(controlLog, 'effort');
   HebiUtils.plotLogs(receiveLog, 'position');
   HebiUtils.plotLogs(receiveLog, 'velocity');
   HebiUtils.plotLogs(receiveLog, 'effort');

   % Plot the end-effectory trajectory and error
   kinematics_analysis( controlLog, controlArm.kin );
   kinematics_analysis( receiveLog, receiveArm.kin );
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   % Feel free to put more plotting code here %
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

