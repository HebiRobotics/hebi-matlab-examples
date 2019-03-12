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

armName = '2-DoF';
armFamily = 'ExampleArm';
hasGasSpring = false;

[ armGroup, armKin, armParams ] = setupArm( armName, armFamily, hasGasSpring );      

gravityVec = armParams.gravityVec;
effortOffset = armParams.effortOffset;
localDir = armParams.localDir;

% Set up Mobile I/O Controller
retryDelayTime = 2.0;
robotName = 'exampleArm';
controllerName = 'mobilePhone';
while true
    try
        controllerGroup = ...
            HebiLookup.newGroupFromNames( robotName, controllerName );
        break;
    catch
        disp(['  Did not find controller: ' robotName '|' controllerName]);
        pause( retryDelayTime );
    end
end

% Get the initial feedback objects that we'll reuse later for the
% controller group.
fbkControllerIO = controllerGroup.getNextFeedbackIO();
fbkControllerMobile = controllerGroup.getNextFeedbackMobile();
latestControllerIO = fbkControllerIO;
latestControllerMobile = fbkControllerMobile;

% Increase feedback frequency since we're calculating velocities at the
% high level for damping.  Going faster can help reduce a little bit of
% jitter for fast motions, but going slower (100 Hz) also works just fine
% for most applications.
armGroup.setFeedbackFrequency(200); 

numArmDOFs = armKin.getNumDoF();
endVelocities = zeros(1, numArmDOFs);
endAccels = zeros(1, numArmDOFs);

% Double the effort gains from their default values, to make the arm more
% sensitive for tracking force.
gains = armGroup.getGains();
% gains.effortKp = 1 * gains.effortKp;
% gains.effortKd = 2 * gains.effortKd;
armGroup.send('gains',gains);

numDoF = armKin.getNumDoF;

% Trajectory generator
minTrajDuration = 0.1; % [sec]
defaultSpeedFactor = 0.9;

armTrajGen = HebiTrajectoryGenerator(armKin);
armTrajGen.setMinDuration( minTrajDuration ); 
armTrajGen.setSpeedFactor( defaultSpeedFactor );

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

    % HOLD POSITION ONLY (Allow rotation around end-effector position)
    gainsInEndEffectorFrame = false;
    damperGains = [10; 0; 10; .0; .0; .0;]; % (N/(m/sec)) or (Nm/(rad/sec))
    springGains = [300; 0; 300; 0; 0; 0];  % (N/m) or (Nm/rad)

%     % HOLD ROTATION ONLY
%     gainsInEndEffectorFrame = true;
%     damperGains = [0; 0; 0; .1; .1; .1;]; % (N/(m/sec)) or (Nm/(rad/sec))
%     springGains = [0; 0; 0; 5; 5; 5];  % (N/m) or (Nm/rad)
 
%     % HOLD POSITION AND ROTATION - BUT ALLOW MOTION ALONG/AROUND Z-AXIS
%     gainsInEndEffectorFrame = true;
%     damperGains = [10; 10; 0; .1; .1; .1;]; % (N/(m/sec)) or (Nm/(rad/sec))
%     springGains = [500; 500; 0; 5; 5; 5];  % (N/m) or (Nm/rad)
%     
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

armFbkFrequency = armGroup.getFeedbackFrequency();
tStart = fbk.time;
tLast = tStart;
tTrajStart = tStart;

firstRun = true;

while ~keys.ESC   
    
    % Gather sensor data from the arm
    fbk = armGroup.getNextFeedback();
    
    tempFbk = controllerGroup.getNextFeedback( ...
                    fbkControllerIO, fbkControllerMobile, 'timeout', 0 );
    if ~isempty(tempFbk)
        latestControllerMobile = fbkControllerMobile;
        latestControllerIO = fbkControllerIO;
    end
    
    % Timekeeping
    t = fbk.time - tStart;
    dt = t - tLast;
    dt = max( dt, 0.5 * 1/armFbkFrequency );
    dt = min( dt, 2.0 * 1/armFbkFrequency );
    tLast = t;
      
    tTraj = fbk.time - tTrajStart;
      
    %%%%%%%%%%%%%%%%%%%%%%%%
    % Gravity Compensation %
    %%%%%%%%%%%%%%%%%%%%%%%%
    
    % Calculate required torques to negate gravity at current position
    gravCompEfforts = armKin.getGravCompEfforts( fbk.position, gravityVec );
    
    % Get Updated Forward Kinematics and Jacobians
    armTipFK = armKin.getFK('endeffector',fbk.position);
    J_armTip = armKin.getJacobian('endeffector',fbk.position);
        
    %%%%%%%%%%%%%%%%%%%%%
    % Impedance Control %
    %%%%%%%%%%%%%%%%%%%%%
    if controllerOn

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
           
    %%%%%%%%%%%%%%%%%%%%
    % Velocity Control %
    %%%%%%%%%%%%%%%%%%%%
    velScale = 0.4;
    velX = velScale * latestControllerIO.a1;
    velZ = velScale * latestControllerIO.a2;
    
    armTipVelCmd = [ velX; 
                     velZ ];
    
    xyzTargetShift = dt * [ velX; 
                              0; 
                            velZ ];
                              
    if firstRun
        xyzTarget = armTipFK(1:3,4);
    end
                              
                              
    xyzTarget = xyzTarget + xyzTargetShift;
    ikPosition = armKin.getIK( 'xyz', xyzTarget, ...
                               'initial', armParams.ikSeedPos );
                                  
    if firstRun || ~controllerOn
        pos = ikPosition;
        vel = endVelocities;
        accel = endAccels;
        firstRun = false;
    elseif exist('armTraj','var')
        tTraj = min( tTraj, armTraj.getDuration() );
        [pos,vel,accel] = armTraj.getState(tTraj);
    else
        pos = ikPosition;
        vel = endVelocities;
        accel = endAccels;
    end
    % cmd.position(armDOFs) = pos;
    cmd.velocity = vel;

    dynamicsComp = armKin.getDynamicCompEfforts( ...
                                    fbk.position, pos, vel, accel );

    
    % Start new trajectory at the current state
    if max(abs( armTipVelCmd )) > 0
        tTrajStart = fbk.time;
        armTraj = armTrajGen.newJointMove( [pos; ikPosition], ...
                                     'Velocities', [vel; endVelocities], ...
                                     'Accelerations', [accel; endAccels]);        
    end
%     J = J_armTip([1,3],:); 
%     jointVelCmd = pinv_damped(J) * armTipVelCmd;
%     
%     cmd.velocity = jointVelCmd';

    armTipFK = armKin.getFK('endeffector',pos);
    endEffectorXYZ = armTipFK(1:3,4);
    endEffectorRotMat = armTipFK(1:3,1:3);
    
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
        
        firstRun = true;
        
        if controllerOn
            disp('Impedance Controller ENABLED.');
        else
            disp('Impedance Controller DISABLED.');
        end
    end
    
    prevKeys = keys;
end

%%
% Stop Logging
if enableLogging  
   hebilog = armGroup.stopLogFull();
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
   kinematics_analysis( hebilog, armKin );
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   % Feel free to put more plotting code here %
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
