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
HebiLookup.setLookupAddresses({'10.10.12.107', '10.10.10.255'});

enableLogging = true;

%%
%%%%%%%%%%%%%%%%%%%%%%%
% Mobile Device Setup %
%%%%%%%%%%%%%%%%%%%%%%%
phoneFamily = 'Receive';
phoneName = 'mobileIO';

resetPoseButton = 'b1';
quitDemoButton = 'b8';
translationScaleSlider = 'a3';
gripForceSlider = 'a6';

while true  
    try
        fprintf('Searching for phone Controller...\n');
        phoneGroup = HebiLookup.newGroupFromNames( ...
                        phoneFamily, phoneName );        
        disp('Phone Found.  Starting up');
        break;
    catch
        pause(1.0);
    end
end


%%
%%%%%%%%%%%%%
% Arm Setup %
%%%%%%%%%%%%%

armName = 'A-2085-06';
armFamily = 'Receive';
hasGasSpring = false;  % If you attach a gas spring to the shoulder for
                       % extra payload, set this to TRUE.

[ arm, params, gripper ] = setupArm( armName, armFamily, hasGasSpring );
arm.plugins = {
    HebiArmPlugins.EffortOffset(params.effortOffset)
};
                             
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
        phoneGroup.startLog('dir',[params.localDir '/logs']);
    end

    % Move to current coordinates
    xyzTarget_init = [0.5 0.0 0.1]';
    rotMatTarget_init = R_y(pi);

    ikPosition = arm.kin.getIK('xyz', xyzTarget_init, ...
                               'so3', rotMatTarget_init, ...
                               'initial', params.ikSeedPos );   
        
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
    fbkPhoneMobile = phoneGroup.getNextFeedbackMobile();
    fbkPhoneIO = phoneGroup.getNextFeedbackIO();

    q = [ fbkPhoneMobile.arOrientationW ...
          fbkPhoneMobile.arOrientationX ...
          fbkPhoneMobile.arOrientationY ...
          fbkPhoneMobile.arOrientationZ ];     
    R_init = HebiUtils.quat2rotMat( q );

    xyz_init = [ fbkPhoneMobile.arPositionX;
                 fbkPhoneMobile.arPositionY; 
                 fbkPhoneMobile.arPositionZ ];
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
%             gripper.send();
        catch
            disp('Could not get robot feedback!');
            break;
        end

        % We get feedback from the phone into the existing structs. The 
        % timeout of 0 means that the method returns immediately and won't
        % wait for feedback. If there was no feedback, the method returns 
        % empty ([]), and the data in the passed in structs does not get
        % overwritten.
        % We do this because the mobile device is typically on wireless and
        % might drop out or be really delayed, in which case we would
        % rather keep running with an old data instead of waiting here for
        % new data.
        hasNewPhoneFbk = ~isempty(phoneGroup.getNextFeedback( ...
            fbkPhoneIO, fbkPhoneMobile, ... % overwrite existing structs
            'timeout', 0 )); % prevent blocking due to bad comms
        
        % Abort goal updates if the phone didn't respond
        if ~hasNewPhoneFbk
            continue;
        end

        
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
        xyzPhoneNew = [ fbkPhoneMobile.arPositionX; ...
                        fbkPhoneMobile.arPositionY; ...
                        fbkPhoneMobile.arPositionZ ];
                    
        xyzTarget = xyzTarget_init + ...
            phoneControlScale * xyzScale .* (R_init' * (xyzPhoneNew - xyz_init));

        q = [ fbkPhoneMobile.arOrientationW ...
              fbkPhoneMobile.arOrientationX ...
              fbkPhoneMobile.arOrientationY ...
              fbkPhoneMobile.arOrientationZ ];     
        rotMatTarget = R_init' * HebiUtils.quat2rotMat( q ) * rotMatTarget_init;


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

