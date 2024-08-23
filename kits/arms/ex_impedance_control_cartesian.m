                      % End-Effector Impedance Control Demo

% Features:     In this example we will implement various hybrid motion-force controllers using the impedance control plugin, 
%               which can be used for a wide variety of applications.
%               Impedance control is BEST SUITED for enabling free, rigid and springy behaviour, along/about each different axis.
%               While this is perfectly useful for:
%               - Having a selectively compliant end-effector,
%               - Switching between fixed and free behaviour to simulate (mostly) rigid constraints, and
%               - Allowing human intervention for automated operations by separating controls across different axes,
%               any applications involving more salient control of the forces (as more complex functions with flexible inputs) 
%               should use our force control plugin. See ex_force_control_demoname.py.
%
%               This comprises the following demos:
%               - Fixed: A task-space pose controller implemented entirely using force control via the (PD) impedance controller.
%               - Cartesian: Locks onto a particular end-effector position while having some compliant orientation.
%               - Gimbal: A gimbal that locks a specific end-effector orientation, while keeping the rest of the arm compliant.
%               - Floor: The end-effector is free to move but can't travel below a virtual floor. To further simulate sliding on the floor, 
%                        see force_control example.
%               - Damping: The end-effector behaves as 3-different damped systems (overdamped, critically damped, and underdamped), 
%                          at 3 different heights.
%
% Requirements:  MATLAB 2013b or higher
%
% Author:        Dave Rollinson
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
% Date:          Oct 2018
% 
% Copyright 2017-2018 HEBI Robotics
% 
% The following example is for the "Cartesian" demo:

%% Setup
clear *;
close all;

HebiLookup.initialize();

%% Load config file
localDir = fileparts(mfilename('fullpath'));
exampleConfigFile = fullfile(localDir, 'config', 'ex_impedance_control_cartesian.cfg.yaml');
exampleConfig = HebiUtils.loadRobotConfig(exampleConfigFile);

% Instantiate the arm kit based on the config files in config/${name}.yaml
% If your kit has a gas spring, you need to uncomment the offset lines
% in the corresponding config file.
arm = createArmFromConfig(exampleConfig);

% Remove the position gains, so that only the commanded torques 
% are moving the arm.
gains = arm.group.getGains;
gains.positionKp = 0 * gains.positionKp;
gains.positionKi = 0 * gains.positionKi;
gains.positionKd = 0 * gains.positionKd;
gains.velocityKp = 0 * gains.velocityKp;
HebiUtils.sendWithRetry(arm.group, 'gains', gains);

% Keyboard input
kb = HebiKeyboard();

% Increase feedback frequency since we're calculating velocities at the
% high level for damping.  Going faster can help reduce a little bit of
% jitter for fast motions, but going slower (100 Hz) also works just fine
% for most applications.
arm.group.setFeedbackFrequency(200); 

enableLogging = true;

% Start background logging 
if enableLogging
   logFile = arm.group.startLog('dir',[localDir '/logs']); 
end

%% Gravity compensated mode
disp('Commanded gravity-compensated zero force to the arm.');
disp('  SPACE - Toggles an impedance controller on/off:');
disp('          ON  - Apply controller based on current position');
disp('          OFF - Go back to gravity-compensated mode');
disp('  ESC - Exits the demo.');

% Impedance Control Gains
% NOTE: The gains corespond to:
% [ trans_x trans_y trans_z rot_x rot_y rot_z ]
%
% Translations and Rotations can be specified in the
% base frame or in the end effector frame.  See code below for
% details.
%

keys = read(kb);
controllerOn = false;
while ~keys.ESC   
    
    % update state and disable position controller
    arm.update();
    arm.send();

    % Check for new key presses on the keyboard
    [keys, diffKeys] = read(kb);
    if diffKeys.SPACE == 1 
        
        % Toggle impedance
        controllerOn = ~controllerOn;
        
        if controllerOn
            disp('Impedance Controller ENABLED.');
            arm.setGoal(arm.state.fbk.position);
        else
            disp('Impedance Controller DISABLED.');
            arm.clearGoal();
        end
        
    end
    
    prevKeys = keys;
end

disp('Stopping Demo...')

%%
% Stop Logging
if enableLogging  
   hebilog = arm.group.stopLogFull();
end

%%
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
