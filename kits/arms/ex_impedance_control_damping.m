% End-Effector Impedance Control Demos
%
% The impedance control examples modify a built-in impedance controller
% plugin to implement various hybrid motion-force controllers that are
% useful for a variety of applications.
%
% Impedance control can be thought of as a virtual spring that computes
% forces/torques to push a point towards a target. The target in this case
% can be static (e.g. stay at a certain point) or dynamic (e.g. improve 
% tracking over a trajectory).
%
% The spring is implemented with pid gains along/about each rotational and
% translational degree of freedom, and it can be selectively enabled for
% any subset of axes. For example, an end-effector position can be fixed
% while users are free to change the orientation, or a virtual wall can
% push a robot away when entering an undesired region.
%
% These examples comprise of the following demos:
%
% - Fixed:     A task-space pose controller implemented entirely using 
%              force control via the (PD) impedance controller.
%
% - Cartesian: Locks onto a particular end-effector position while keeping
%              the orientation compliant.
%
% - Gimbal:    A gimbal that locks a specific end-effector orientation, 
%              while keeping the end effecotr position compliant.
%
% - Damping:  The end-effector behaves as 3-different damped systems
%             (overdamped, critically damped, and underdamped), at 3 
%             different heights.
%
% Note that the first 3 examples only differ by the gains in the
% corresponding config files.
% 
% The following example is the 'Damping' demo.

% Copyright 2017-2024 HEBI Robotics

%% Setup
clear *;
close all;
HebiLookup.initialize();

% Demo Settings
enableLogging = true;

%% Load config and setup components
config = HebiUtils.loadRobotConfig('config/ex_impedance_control_damping.cfg.yaml');
arm = HebiArm.createFromConfig(config);
impedanceController = arm.plugins.impedanceController;
userData = config.userData;

% Increase feedback frequency since we're calculating velocities at the
% high level for damping.  Going faster can help reduce a little bit of
% jitter for fast motions, but going slower (100 Hz) also works just fine
% for most applications.
arm.group.setFeedbackFrequency(200); 

% The impedance controller is based on the position/velocity commands. It
% typically works in combination with the joint-level controllers, but for
% this demo we are only interested in the impedance controller effects. We
% can effectively disable the joint controllers by setting the
% corresponding pid gains to zero. We also double the effort kp in order to
% make the arm more sensitive.
%
% The impedance controller itself has a separate set of pid gains that
% correspond to [ trans_x trans_y trans_z rot_x rot_y rot_z ], as well as a
% setting to specify whether they are applied in the base frame or the
% end-effector frame.
gains = arm.group.getGains();
gains.positionKp = 0 * gains.positionKp;
gains.positionKi = 0 * gains.positionKi;
gains.positionKd = 0 * gains.positionKd;
gains.velocityKp = 0 * gains.velocityKp;
gains.effortKp   = 2 * gains.effortKp;
HebiUtils.sendWithRetry(arm.group, 'gains', gains);

%% Start optional background logging
if enableLogging
    logFile = arm.group.startLog('dir', 'logs');
end

%% Demo - Impedance Control
disp('Commanded gravity-compensated zero force to the arm.');
disp('  SPACE - Toggles an impedance controller on/off:');
disp('          ON - Apply controller based on current height, ')
disp('               End-Effector is free to move along Z')
disp('               LOW  - overdamped');
disp('               MID  - critically damped');
disp('               HIGH - underdamped');
disp('          OFF - Go back to gravity-compensated mode');
disp('  ESC - Exits the demo.');

% Main demo loop
prevMode = -1;
kb = HebiKeyboard();
keys = read(kb);
controllerOn = false;
while ~keys.ESC   
    
    % update state and disable position controller
    arm.update();
    arm.send();

    % Check for new key presses on the keyboard
    [keys, diffKeys] = read(kb);
    if diffKeys.SPACE == 1 
        
        % Toggle impedance controller
        controllerOn = ~controllerOn;
        
        if controllerOn
            % The impedance plugin works based on the commanded position,
            % so we set the position to wherever it was at enablement.
            disp('Impedance Controller ENABLED.');
            arm.setGoal(arm.state.fbk.position);
        else
            % Pure gravity compensation with unrestricted motion
            disp('Impedance Controller DISABLED.');
            arm.clearGoal();
        end
        
    end

    % When impedance control is on, we set the gains based on the height of
    % the end-effector.
    if controllerOn

        % Assign mode based on the end-effector z position (height)
        % (1) low  - overdamped
        % (2) mid  - critically damped
        % (3) high - underdamped
        zHeight = arm.state.T_endEffector(3,4);
        mode = find(userData.lower_limits > zHeight, 1);
        if isempty(mode)
            mode = 3;
        end

        % Assign new gains on mode changes
        if mode ~= prevMode
            switch mode

                case 1
                    disp('Mode: overdamped')
                    impedanceController.Kp = userData.overdamped_kp';
                    impedanceController.Kd = userData.overdamped_kd';

                case 2
                    disp('Mode: critically damped');
                    impedanceController.Kp = userData.critically_damped_kp';
                    impedanceController.Kd = userData.critically_damped_kd';

                case 3
                    disp('Mode: underdamped');
                    impedanceController.Kp = userData.underdamped_kp';
                    impedanceController.Kd = userData.underdamped_kd';

            end
            prevMode = mode;
        end

    end

end

disp('Stopping Demo.')

%% Stop Logging
if enableLogging  
   hebilog = arm.group.stopLogFull();
end

%% Analysis of logged data
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
