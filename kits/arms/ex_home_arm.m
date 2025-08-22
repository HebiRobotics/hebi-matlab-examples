% Arm Home Demo
%
% Features:      Simple demo that moves an arm to the home position.
%
% Requirements:  MATLAB 2013b or higher
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
% Date:          Aug 2025

% Copyright 2017-2025 HEBI Robotics

%% Setup
% Initialize the interface for network connected modules
% The lookup class is used to find modules on the network
clear *;
close all;
HebiLookup.initialize();

% Demo Settings
enableLogging = true;

%% Load config and setup components
% Config file contains the necessary information to set up the arm, such as
% the family name, module names, path to HRDF file for the kinematics, etc.
% 
% By default, this example uses a 6-DoF arm with X-Series modules
% You can edit the config file to match your robot's configuration
% For example, if you have a 5-DoF T-Series arm, replace the config file
% with "config/A-2580-05.cfg.yaml"

config = HebiUtils.loadRobotConfig('config/A-2085-06.cfg.yaml');

% Set up an arm based on config information (names, kinematics, etc.)
arm = HebiArm.createFromConfig(config);

% Settings can be overriden programmatically if desired
arm.group.setFeedbackFrequency(200);

%% Demo - Go to home position
% (Optional) Record all data from this point on
if enableLogging
   logFile = arm.group.startLog('dir', 'logs'); 
end

% Read home position from user data
home = config.userData.home_position;

% Go to the goal
arm.setGoal(home);
while ~arm.isAtGoal()
    arm.update();
    arm.send();
end

% (Optional) Convert recorded data up to this point
if enableLogging  
   hebilog = arm.group.stopLogFull();
end

%% Analysis of logged data
% Plotting
if enableLogging
   
   % Plot tracking / error from the joints in the arm.  Note that there
   % will not by any 'error' in tracking for position and velocity, since
   % this example only commands effort.
   HebiUtils.plotLogs(hebilog, 'position');
   HebiUtils.plotLogs(hebilog, 'velocity');
   HebiUtils.plotLogs(hebilog, 'effort');
   
   % Plot the end-effectory trajectory and error
   kinematics_analysis( hebilog, arm.kin );
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   % Feel free to put more plotting code here %
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

