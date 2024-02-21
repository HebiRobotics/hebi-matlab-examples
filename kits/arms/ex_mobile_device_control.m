% 6-DoF Arm (w/ Optional Gripper) Tele-Op Demo 
%
% Features:      Mobile App input to control motions of arm and gripper
%
% Requirements:  MATLAB 2013b or higher
%                Android or iOS device running HEBI Mobile I/O that
%                supports ARCore / ARKit.
%
% Author:        Dave Rollinson
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
% Date:          Oct 2018

% Copyright 2017-2018 HEBI Robotics

%% Setup
clear *;
close all;

HebiLookup.initialize();

enableLogging = true;

%%
%%%%%%%%%%%%%%%%%%%%%%%
% Mobile Device Setup %
%%%%%%%%%%%%%%%%%%%%%%%
mobileIO = HebiMobileIO.findDevice('HEBI', 'mobileIO');
mobileIO.initializeUI();
mobileIO.setAxisValue([3 6], [-1 1]);
mobileIO.setButtonIndicator([1 8], true);
mobileIO.addText('B1 - Reset/re-align pose');
mobileIO.addText('A3 - Scale translation commands');
mobileIO.addText('A6 - Gripper Open/Close');
mobileIO.addText('B8 - Quit');

resetPoseButton = 'b1';
quitDemoButton = 'b8';
translationScaleSlider = 'a3';
gripForceSlider = 'a6';

%%
%%%%%%%%%%%%%
% Arm Setup %
%%%%%%%%%%%%%

% Instantiate the arm kit based on the config files in config/${name}.yaml
% If your kit has a gas spring, you need to uncomment the offset lines
% in the corresponding config file.
[ arm, params, gripper ] = setupArm( 'A-2240-06G' );
                             
disp('  ');
disp('Arm end-effector is now following the mobile device pose.');
disp('The control interface has the following commands:');
disp('  B1 - Reset/re-align poses.');
disp('       This takes the arm to home and aligns with mobile device.');
disp('  A3 - Scale down the translation commands to the arm.');
disp('       Sliding all the way down means the end-effector only rotates.');
disp('  A6 - Control the gripper (if the arm has gripper).');
disp('       Sliding down closes the gripper, sliding up opens.');
disp('  B8 - Quits the demo.');


%% Startup
abortFlag = false;
while ~abortFlag
    
    xyzScale = [1 1 2]';

    % Start background logging
    if enableLogging
        arm.group.startLog('dir',[params.localDir '/logs']);
    end

    % Move to current coordinates
    xyzTarget_init = [0.5 0.0 0.1]';
    rotMatTarget_init = R_y(pi);

    ikPosition = arm.kin.getIK('xyz', xyzTarget_init, ...
                               'so3', rotMatTarget_init, ...
                               'initial', params.ik_seed_pos );
        
    % Slow trajectory timing for the initial move to home position   
    arm.trajGen.setSpeedFactor( 0.5 );
    arm.trajGen.setMinDuration( 1.0 );
    
    % Move to initial position
    arm.update();
    arm.clearGoal(); % in case we run only this section
    arm.setGoal(ikPosition);
    while ~arm.isAtGoal
        arm.update();
        arm.send();
    end
    
    % Grab initial pose from phone
    mobileIO.update();
    R_init = mobileIO.getArOrientation();
    xyz_init = mobileIO.getArPosition();
    xyzPhoneNew = xyz_init;

    % Set trajectories to normal speed for following mobile input
    arm.trajGen.setSpeedFactor( 1.0 );
    arm.trajGen.setMinDuration( 0.5 ); % (acts as a 'low-pass' for user input)
    goalPosition = ikPosition;
    while ~abortFlag

        %%%%%%%%%%%%%%%%%%%
        % Gather Feedback %
        %%%%%%%%%%%%%%%%%%%
        try
            arm.update();
            arm.send();
            gripper.send();
        catch
            disp('Could not get robot feedback!');
            break;
        end

        % We use a non-blocking timeout w/ manual update checks so we can
        % handle common wireless delays/outages without breaking. We only
        % exit if mobileIO hasn't responded for several seconds, which
        % indicates a more significant problem.
        [hasNewPhoneFbk, timeSinceLastFeedback] = mobileIO.update('timeout', 0);
        if timeSinceLastFeedback > 5
            error('mobileIO has timed out. Stopping demo.');
        end
        
        % Abort goal updates if the phone didn't respond
        if ~hasNewPhoneFbk
            continue;
        end
        fbkPhoneIO = mobileIO.getFeedbackIO();
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Read/Map Joystick Inputs %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Check for restart command
        if fbkPhoneIO.(resetPoseButton)
            break;
        end
        
        % Check for quit command
        if fbkPhoneIO.(quitDemoButton)
            abortFlag = true;
            break;
        end
        
        % Map [-1,+1] slider to [0,1] range such that down is close
        if ~isempty(gripper)
            gripper.setState((fbkPhoneIO.(gripForceSlider) - 1) / -2);
        end
        
        % Parameter to limit XYZ Translation of the arm if a slider is
        % pulled down.  Pulling all the way down resets translation.
        phoneControlScale = fbkPhoneIO.(translationScaleSlider);
        if phoneControlScale < 0
            xyz_init = xyzPhoneNew;
        end

        % Pose Information for Arm Control
        xyzPhoneNew = mobileIO.getArPosition();
        xyzTarget = xyzTarget_init + ...
            phoneControlScale * xyzScale .* (R_init' * (xyzPhoneNew - xyz_init));   
        rotMatTarget = R_init' * mobileIO.getArOrientation() * rotMatTarget_init;

        %%%%%%%%%%%%%%%
        % Arm Control %
        %%%%%%%%%%%%%%%
        % Force elbow up config
        seedPosIK = arm.state.cmdPos;
        seedPosIK(3) = abs(seedPosIK(3));

        % Find target using inverse kinematics
        ikPosition = arm.kin.getIK('xyz', xyzTarget, ...
                                   'SO3', rotMatTarget, ...
                                   'initial', seedPosIK, ...
                                   'MaxIter', 50 ); 
                               
        % Ignore 'changes' that are within the noise of the input device
        hasGoalChanged = any(abs(ikPosition - goalPosition) > 0.01); % [rad]

        % Start new trajectory at the current state
        if hasGoalChanged && arm.state.trajTime > 0.1 % limit replanning to 10 Hz (optional)
            arm.setGoal(ikPosition); 
            goalPosition = ikPosition;
        end

    end

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

