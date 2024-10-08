% End-Effector Impedance Control Demo
%
% Features:      Demo where the arm can be interacted with and moved around
%                while in a zero-force gravity-compensated mode, and an
%                impedance controller can be turned on and off where the
%                end-effector is controlled based on virtual springs and
%                dampers in Cartesian space.  Multiple spring/damper
%                configurations are selectable in the code below.
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

% Instantiate the arm kit based on the config files in config/${name}.yaml
% If your kit has a gas spring, you need to uncomment the offset lines
% in the corresponding config file.
[ arm, params ] = setupArm( 'A-2085-06' );
    
% Append the impedance controller plugin. You can also define it in the 
% config file so it gets applied to all demos.
impedance = HebiArmPlugins.ImpedanceController();
arm.plugins{end+1} = impedance;

% Increase feedback frequency since we're calculating velocities at the
% high level for damping.  Going faster can help reduce a little bit of
% jitter for fast motions, but going slower (100 Hz) also works just fine
% for most applications.
arm.group.setFeedbackFrequency(200); 

% Remove the position gains, so that only the commanded torques 
% are moving the arm.
gains = params.gains;
gains.positionKp = 0 * gains.positionKp;
gains.positionKi = 0 * gains.positionKi;
gains.positionKd = 0 * gains.positionKd;
gains.velocityKp = 0 * gains.velocityKp;
HebiUtils.sendWithRetry(arm.group, 'gains', gains);

enableLogging = true;

% Keyboard input
kb = HebiKeyboard();

% Start background logging 
if enableLogging
   logFile = arm.group.startLog('dir',[params.localDir '/logs']); 
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
% UNCOMMENT THE GAINS YOU WANT TO USE FOR A GIVEN RUN, AND COMMENT OUT ALL
% THE OTHER GAINS.

    % HOLD POSITION ONLY (Allow rotation around end-effector position)
    impedance.gainsInEndEffectorFrame = true; 
    impedance.Kp = [1000; 1000; 1000; 0; 0; 0];  % (N/m) or (Nm/rad)
    impedance.Kd = [10; 10; 10; .0; .0; .0;]; % (N/(m/sec)) or (Nm/(rad/sec))
 
%     % HOLD ROTATION ONLY
%     impedance.gainsInEndEffectorFrame = true;
%     impedance.Kp = [0; 0; 0; 5; 5; 5];  % (N/m) or (Nm/rad)
%     impedance.Kd = [0; 0; 0; .1; .1; .1;]; % (N/(m/sec)) or (Nm/(rad/sec))
  
%     % HOLD POSITION AND ROTATION - BUT ALLOW MOTION ALONG/AROUND Z-AXIS 
%     % OF THE END EFFECTOR
%     impedance.gainsInEndEffectorFrame = true; 
%     impedance.Kp = [500; 500; 0; 5; 5; 0];  % (N/m) or (Nm/rad)
%     impedance.Kd = [10; 10; 0; .1; .1; 0;]; % (N/(m/sec)) or (Nm/(rad/sec))
    
%     % HOLD POSITION AND ROTATION - BUT ALLOW MOTION IN BASE FRAME XY-PLANE
%     impedance.gainsInEndEffectorFrame = false;
%     impedance.Kp = [0; 0; 1000; 5; 5; 5];  % (N/m) or (Nm/rad)
%     impedance.Kd = [0; 0; 10 ; .1; .1; .1;]; % (N/(m/sec)) or (Nm/(rad/sec))
 

keys = read(kb);
controllerOn = false;
while ~keys.ESC   
    
    % update state and disable position controller
    arm.update();
    arm.state.cmdPos = []; 
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
