% Arm Gravity Compensation Demo
%
% Features:      Demo where the arm can be interacted with and moved around
%                while in a zero-force gravity-compensated mode.
%
% Requirements:  MATLAB 2013b or higher
%
% Author:        Dave Rollinson
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
% Date:          Oct 2018

% Copyright 2017-2018 HEBI Robotics

%% Setup
clear *;
close all;

HebiLookup.initialize();

%% Load config file
% Instantiate the arm kit based on the config files in config/${name}.yaml
% If your kit has a gas spring, you need to uncomment the offset lines
% in the corresponding config file.
arm = HebiArm.createFromConfig('./config/ex_gravity_compensation.cfg.yaml');

enableLogging = true;

% Start background logging 
if enableLogging
   logFile = arm.group.startLog('dir', './logs'); 
end

%% Gravity compensated mode
disp('Commanded gravity-compensated zero torques to the arm.');
disp('Press ESC to stop.');

% Keyboard input
kb = HebiKeyboard();
keys = read(kb);
while ~keys.ESC   
   keys = read(kb);
   
   arm.update();
   arm.send();

end

%%
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
