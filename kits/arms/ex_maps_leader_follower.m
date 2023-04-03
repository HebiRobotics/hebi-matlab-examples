% R-Series Snake Arm Tele-Operated with 
%
% Author:        Andrew Willig
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
% Date:          Apr 2021

% Copyright 2021-2022 HEBI Robotics

%% Setup
clear *;
close all;

HebiLookup.initialize();

enableLogging = false;


kb = HebiKeyboard();

%%

resetPoseButton = 'TAB';
quitDemoButton = 'ESC';
pauseButton = 'SPACE';
% translationScaleSlider = 'a3';
toolSpinSlider = 'UP';

% cmdIO = IoCommandStruct();
% cmdIO.a6 = 0.00001;  % Set slider 'a3' to snap to center.

joyDeadzone = 0.20;

%%
%%%%%%%%%%%%%%%%%%
% MAPS Arm Setup %
%%%%%%%%%%%%%%%%%%

[ mapsArm, ~ ] = setupArm_MAPS();
mapsGroup = mapsArm.group;
mapsKin = mapsArm.kin;

%%
%%%%%%%%%%%%%
% Arm Setup %
%%%%%%%%%%%%%

armName = 'A-2240-06';
armFamily = 'Arm';
hasGasSpring = false;  % If you attach a gas spring to the shoulder for
                       % extra payload, set this to TRUE.

[ arm, params ] = setupArm( armName, armFamily, hasGasSpring );
arm.plugins = {
	HebiArmPlugins.EffortOffset(params.effortOffset)  
};

localDir = fileparts(mfilename('fullpath'));
params.localDir = localDir;
                             
disp('  ');
disp('Arm end-effector is now following the MAPS arm pose.');
disp('The control interface has the following commands:');
disp('  TAB - Reset/re-align poses.');
disp('        This takes the arm to home and aligns with MAPS.')
disp('  SPACE - Pause the demo.');
disp('  ESC - Quits the demo.');

%% Startup
arm.update();
mapsArm.update();

demoPaused = false;
abortFlag = false;

while ~abortFlag
    
    xyzScale = [1 1 1]';

    % Start background logging
    if enableLogging
        arm.group.startLog('dir',[params.localDir '/logs']);
        mapsGroup.startLog('dir',[params.localDir '/logs']);
    end

    % Move to current coordinates
    xyzTarget_init = [0.4 0.00 0.1]';
    rotMatTarget_init = R_y(pi);

    ikPosition = arm.kin.getIK('xyz', xyzTarget_init, ...
                               'so3', rotMatTarget_init, ...
                               'initial', params.ikSeedPos );   
        
    % Slow trajectory timing for the initial move to home position   
    arm.trajGen.setSpeedFactor( 1.0 );
    arm.trajGen.setMinDuration( 5.0 );
    
    % Move to initial position
    arm.update();
    arm.clearGoal(); % in case we run only this section
    arm.setGoal(ikPosition);
    disp('Moving arm to home position.  Hold MAPS in desired home position.');
    while ~arm.isAtGoal
        arm.update();
        arm.send('led','b');  % Set LEDs blue while homingo
        mapsArm.send('led','b');
    end
    disp('Homing Complete!');

    % Grab initial pose from MAPS 
    mapsFbk = mapsGroup.getNextFeedbackFull();
    
    mapsT_init = mapsKin.getFK('endeffector', mapsFbk.position);
    R_init = mapsT_init(1:3,1:3);
    xyz_init = mapsT_init(1:3,4);
    xyzMapsNew = xyz_init;

    mainTime = tic;
    tLast = toc(mainTime);
    
    % Set trajectories to normal speed for following mobile input
    arm.trajGen.setSpeedFactor( 1.0 );
    arm.trajGen.setMinDuration( 0.4 ); % (acts as a 'low-pass' for user input)
    goalPosition = ikPosition;
    [keys, diffKeys] = read(kb);
    while ~abortFlag

        % Timekeeping
        t = toc(mainTime);
        dt = t - tLast;
        tLast = t;
        
        %%%%%%%%%%%%%%%%%%%
        % Gather Feedback %
        %%%%%%%%%%%%%%%%%%%
        try
            arm.update();
            if ~demoPaused
                arm.send('led','g');   % Set LEDs green while operating
                mapsArm.send('led','g');
            else
                arm.send('led','m');   % Set LEDs green while operating
                mapsArm.send('led','m');
            end
        catch
            disp('Could not get robot feedback!');
            break;
        end
        
        try
            mapsFbk = mapsGroup.getNextFeedbackFull();
        catch
            disp('Could not get MAPS feedback!');
            break;
        end
        
        [keys, diffKeys] = read(kb);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Read/Map Joystick Inputs %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Check for restart command
        if diffKeys.(resetPoseButton)
            break;
        end
        
        % Check for pause command
        if diffKeys.(pauseButton) > 0
            if ~demoPaused
                demoPaused = true;
                disp('Pausing Demo.  Press SPACE to resume.');
            else
                demoPaused = false;
                disp('Resuming Demo.');
            end

        end
        
        % Check for quit command
        if diffKeys.(quitDemoButton)
            abortFlag = true;
            break;
        end

        % Pose Information for Arm Control
        mapsT = mapsKin.getFK('endeffector', mapsFbk.position);
        xyzMapsNew = mapsT(1:3,4);
        R_maps = mapsT(1:3,1:3);
                    
%         xyzTarget = xyzTarget_init + ...
%             phoneControlScale * xyzScale .* (xyzMapsNew - xyz_init);
        
        xyzTarget = xyzTarget_init + ...
            xyzScale .* (xyzMapsNew - xyz_init);
        
        rotMatTarget = R_maps * R_init' * rotMatTarget_init;

        %%%%%%%%%%%%%%%
        % Arm Control %
        %%%%%%%%%%%%%%%
        % Update IK seed positions
        seedPosIK = arm.state.cmdPos;

        % Find target using inverse kinematics
        ikPosition = arm.kin.getIK('xyz', xyzTarget, ...
                                   'SO3', rotMatTarget, ...
                                   'initial', seedPosIK, ...
                                   'MaxIter', 50 );                     
                             
        % Ignore 'changes' that are within the noise of the input device
        hasGoalChanged = any(abs(ikPosition - goalPosition) > 0.01); % [rad]
        % hasGoalChanged = true;
        
        % Start new trajectory at the current state
        if hasGoalChanged && ~demoPaused
            arm.setGoal(ikPosition); 
            goalPosition = ikPosition;
        end

    end

end

arm.send('led',[]);
mapsArm.send('led',[]);

%%
if enableLogging
    
   hebilog = arm.group.stopLogFull();
   
   
   % Plot tracking / error from the joints in the arm.  Note that there
   % will not by any 'error' in tracking for position and velocity, since
   % this example only commands effort.
   for i = 1:2
       HebiUtils.plotLogs(hebilog, 'position');
       HebiUtils.plotLogs(hebilog, 'velocity');
       HebiUtils.plotLogs(hebilog, 'effort');
   
       % Plot the end-effectory trajectory and error
       kinematics_analysis( hebilog, arm.kin );
   end
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   % Feel free to put more plotting code here %
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

