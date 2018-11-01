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

armName = '6-DoF + gripper';
armFamily = 'Arm';
hasGasSpring = true;

[ armGroup, armKin, armParams ] = setupArm( armName, armFamily, hasGasSpring );      

gravityVec = armParams.gravityVec;
effortOffset = armParams.effortOffset;
localDir = armParams.localDir;

% Increase feedback frequency since we're calculating velocities at the
% high level for damping.  Going faster can help reduce a little bit of
% jitter for fast motions, but going slower (100 Hz) also works just fine
% for most applications.
armGroup.setFeedbackFrequency(200); 

% Double the effort gains from their default values, to make the arm more
% sensitive for tracking force.
gains = armGroup.getGains();
gains.effortKp = 2 * gains.effortKp;
armGroup.send('gains',gains);

numDoF = armKin.getNumDoF;

enableLogging = true;

% Start background logging 
if enableLogging
   logFile = armGroup.startLog('dir',[localDir '/logs']); 
end

%% Gravity compensated mode
cmd = CommandStruct();

% Keyboard input
kb = HebiKeyboard();
keys = read(kb);

disp('Commanded gravity-compensated zero force to the arm.');
disp('  SPACE - Toggles an impedance controller on/off:');
disp('          ON  - Apply controller based on current position');
disp('          OFF - Go back to gravity-compensated mode');
disp('  ESC - Exits the demo.');

% Impendance Control Gains
% NOTE: The gains corespond to:
% [ trans_x trans_y trans_z rot_x rot_y rot_z ]
%
% Translations and Rotations can be specified in the
% base frame or in the end effector frame.  See code below for
% details.
%
% UNCOMMENT THE GAINS YOU WANT TO USE FOR A GIVEN RUN, AND COMMENT OUT ALL
% THE OTHER GAINS.

%     % HOLD POSITION ONLY (Allow rotation around end-effector position)
%     gainsInEndEffectorFrame = true;
%     damperGains = [5; 5; 5; .0; .0; .0;]; % (N/(m/sec)) or (Nm/(rad/sec))
%     springGains = [500; 500; 500; 0; 0; 0];  % (N/m) or (Nm/rad)

%     % HOLD ROTATION ONLY
%     gainsInEndEffectorFrame = true;
%     damperGains = [0; 0; 0; .1; .1; .1;]; % (N/(m/sec)) or (Nm/(rad/sec))
%     springGains = [0; 0; 0; 5; 5; 5];  % (N/m) or (Nm/rad)
 
    % HOLD POSITION AND ROTATION - BUT ALLOW MOTION ALONG/AROUND Z-AXIS
    gainsInEndEffectorFrame = true;
    damperGains = [10; 10; 0; .1; .1; .1;]; % (N/(m/sec)) or (Nm/(rad/sec))
    springGains = [500; 500; 0; 5; 5; 5];  % (N/m) or (Nm/rad)
    
%     % HOLD POSITION AND ROTATION - BUT ALLOW MOTION IN BASE FRAME XY-PLANE
%     gainsInEndEffectorFrame = false;
%     damperGains = [0; 0; 5; .1; .1; .1;]; % (N/(m/sec)) or (Nm/(rad/sec))
%     springGains = [0; 0; 500; 5; 5; 5];  % (N/m) or (Nm/rad)

% Get the current location of the end effector
fbk = armGroup.getNextFeedback();
armTipFK = armKin.getFK('endeffector',fbk.position);
endEffectorXYZ = armTipFK(1:3,4);
endEffectorRotMat = armTipFK(1:3,1:3);
    
controllerOn = false;

% Velocity commands
armCmdJointAngs = fbk.position;
armCmdJointVels = zeros(1,numDoF);

while ~keys.ESC   
    
    % Gather sensor data from the arm
    fbk = armGroup.getNextFeedback();
    
    %%%%%%%%%%%%%%%%%%%%%%%%
    % Gravity Compensation %
    %%%%%%%%%%%%%%%%%%%%%%%%
    
    % Calculate required torques to negate gravity at current position
    gravCompEfforts = armKin.getGravCompEfforts( fbk.position, gravityVec );
             
    %%%%%%%%%%%%%%%%%%%%%
    % Impedance Control %
    %%%%%%%%%%%%%%%%%%%%%
    if controllerOn
        % Get Updated Forward Kinematics and Jacobians
        armTipFK = armKin.getFK('endeffector',fbk.position);
        J_armTip = armKin.getJacobian('endeffector',fbk.position);

        % Calculate Impedence Control Wrenches and Appropraite Joint Torque
        springWrench = zeros(6,1);
        damperWrench = zeros(6,1);

        % Linear error is easy
        xyzError = endEffectorXYZ - armTipFK(1:3,4);

        % Rotational error involves calculating axis-angle from the
        % resulting error in S03 and providing a torque around that axis.
        errorRotMat = endEffectorRotMat * armTipFK(1:3,1:3)';
        [axis, angle] = HebiUtils.rotMat2axAng( errorRotMat );
        rotErrorVec = angle * axis;

        if gainsInEndEffectorFrame
            xyzError = armTipFK(1:3,1:3)' * xyzError;
            rotErrorVec = armTipFK(1:3,1:3)' * rotErrorVec;
        end

        posError = [xyzError; rotErrorVec];
        velError = J_armTip * (armCmdJointVels - fbk.velocity)';     

        springWrench(1:3) = springGains(1:3) .* posError(1:3); % linear force
        springWrench(4:6) = springGains(4:6) .* posError(4:6); % rotational torque

        if gainsInEndEffectorFrame
            springWrench(1:3) = armTipFK(1:3,1:3) * springWrench(1:3);
            springWrench(4:6) = armTipFK(1:3,1:3) * springWrench(4:6);
        end

        damperWrench(1:3) = damperGains(1:3) .* velError(1:3); % linear damping
        damperWrench(4:6) = damperGains(4:6) .* velError(4:6); % rotational damping

        impedanceEfforts = J_armTip' * (springWrench + damperWrench);  
    else
        impedanceEfforts = zeros(numDoF,1);
    end
    
    % Add all the different torques together
    cmd.effort = gravCompEfforts + impedanceEfforts' + effortOffset;

    % Send to robot
    armGroup.send(cmd);

    % Check for new key presses on the keyboard
    keys = read(kb);

    % Toggle impedance
    if keys.SPACE == 1 && prevKeys.SPACE == 0      
        controllerOn = ~controllerOn;
        
        armTipFK = armKin.getFK('endeffector',fbk.position);
        endEffectorXYZ = armTipFK(1:3,4);
        endEffectorRotMat = armTipFK(1:3,1:3);
        
        if controllerOn
            disp('Impedance Controller ENABLED.');
        else
            disp('Impedance Controller DISABLED.');
        end
    end
    
    prevKeys = keys;
end

%%
if enableLogging
    
   hebilog = armGroup.stopLogFull();
   
   % Plot tracking / error from the joints in the arm.  Note that there
   % will not by any 'error' in tracking for position and velocity, since
   % this example only commands effort.
   HebiUtils.plotLogs(hebilog, 'position');
   HebiUtils.plotLogs(hebilog, 'velocity');
   HebiUtils.plotLogs(hebilog, 'effort');
   
   % Plot the end-effectory trajectory and error
   kinematics_analysis( hebilog, armKin );
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   % Feel free to put more plotting code here %
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
