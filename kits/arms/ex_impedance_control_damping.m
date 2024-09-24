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
% The following example is for the "Damping" demo:

%% Setup
clear *;
close all;

HebiLookup.initialize();

%% Load config file
localDir = fileparts(mfilename('fullpath'));
exampleConfigFile = fullfile(localDir, 'config', 'ex_impedance_control_damping.cfg.yaml');
exampleConfig = HebiUtils.loadRobotConfig(exampleConfigFile);

% Instantiate the arm kit based on the config files in config/${name}.yaml
% If your kit has a gas spring, you need to uncomment the offset lines
% in the corresponding config file.
arm = HebiArm.createFromConfig(exampleConfig);

% Remove the position gains, so that only the commanded torques 
% are moving the arm.
gains = arm.group.getGains;
gains.positionKp = 0 * gains.positionKp;
gains.positionKi = 0 * gains.positionKi;
gains.positionKd = 0 * gains.positionKd;
gains.velocityKp = 0 * gains.velocityKp;
HebiUtils.sendWithRetry(arm.group, 'gains', gains);

% Retreive the impedance control plugin using the "name" field in the config file
impedancePlugin = arm.plugins.impedanceController;
% impedancePlugin = arm.getPluginByType('HebiArmPlugins.ImpedanceController'); % "type" field can also be used

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

% Control Variables

% Meters above the base for overdamped, critically damped, and underdamped cases respectively
lowerLimits = exampleConfig.userData.lower_limits;
% State variable for current mode: 0 for overdamped, 1 for crtically damped, 2 for underdamped, -1 for free
mode = -1;
prevmode = -1;

% Arrange different gains in an ordered list
dampingKp = [exampleConfig.userData.overdamped_kp;
              exampleConfig.userData.critically_damped_kp; 
              exampleConfig.userData.underdamped_kp];
dampingKd = [exampleConfig.userData.overdamped_kd; 
              exampleConfig.userData.critically_damped_kd; 
              exampleConfig.userData.underdamped_kd];

keys = read(kb);
controllerOn = false;
while ~keys.ESC   
    
    % update state and disable position controller
    arm.update();
    arm.send();

    % Set and unset impedance mode when button is pressed and released, respectively
    [keys, diffKeys] = read(kb);
    if diffKeys.SPACE == 1 
        
        % Toggle impedance
        controllerOn = ~controllerOn;
        
        if controllerOn

            disp('Impedance Controller ENABLED.');
            arm.setGoal(arm.state.fbk.position);

        else
            disp('Impedance Controller DISABLED.');
        end
        
    end

    if controllerOn

        % Use forward kinematics to calculate pose of end-effector
        eePoseCurr = arm.kin.getFK('endEffector', arm.group.getNextFeedback().position);

        % Assign mode based on current position
        for i = 1:length(lowerLimits)
            if eePoseCurr(3,4) > lowerLimits(i)
                mode = i-1;
            end
        end
        
        % Change gains only upon mode switches
        if mode ~= prevmode && mode >= 0
            % Set the gains based on the new mode
            impedance_plugin.Kp = dampingKp(mode+1,:)';
            impedance_plugin.Kd = dampingKd(mode+1,:)';
            
            fprintf('Mode: %d\n', mode);
        end

        % Update prevmode to current mode
        prevmode = mode;

    else

        arm.clearGoal();
        mode = -1; %  Free
        prevmode = -1;

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
