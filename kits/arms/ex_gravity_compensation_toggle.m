                   % Arm Gravity Compensation Toggle Demo
%
% Features:      Demo where the arm can be interacted with and moved around
%                while in a zero-force gravity-compensated mode, which can be toggled on and off.
%
% Requirements:  MATLAB 2013b or higher
%
% Author:        Dave Rollinson
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
% Date:          Oct 2018

% Copyright 2017-2024 HEBI Robotics

%% Setup
clear *;
close all;
HebiLookup.initialize();

% Demo Settings
enableLogging = true;
            
%% Set up arm from config
arm = HebiArm.createFromConfig('config/ex_gravity_compensation_toggle.cfg.yaml');

% Get a reference to the grav comp plugin so we can enable/disable it
gravCompPlugin = arm.getPluginByType('GravityCompensationEffort');

%% Instructions
disp('Commanded gravity-compensated zero torques to the arm.');
disp('  SPACE - Toggles gravity compensation on/off:');
disp('  ESC - Exits the demo.');
disp(' ')
disp(['Gravity Compensation: ' num2str(gravCompPlugin.enabled)])

%% Start optional background logging
if enableLogging
   logFile = arm.group.startLog('dir', 'logs');
end

%% Demo - Toggle Grav Comp
% Keyboard input
kb = HebiKeyboard();
keys = read(kb);
while ~keys.ESC   

   % Without a set goal, only the grav comp plugin is actively working.
   % Thus, when we have an empty control loop, the arm is in
   % grav-comp awaiting further instructions.
   %
   % Note that no robotic system is modelled perfectly, so some amount of
   % drift is normal. Real systems often overlay position control, and
   % only enable true grav-comp when a user is actively guiding the robot.
   % This can e.g. be determined by a button on the end effector.
   arm.update();
   arm.send();

   % Toggle grav comp when the key is pressed down
   [keys, diffKeys] = read(kb);
   if diffKeys.SPACE == 1
       gravCompPlugin.enabled = ~gravCompPlugin.enabled;
       disp(['Gravity Compensation: ' num2str(gravCompPlugin.enabled)])
   end

end

%% Analysis of logged data
if enableLogging
    
   hebilog = arm.group.stopLogFull();
   
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
