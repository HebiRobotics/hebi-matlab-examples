% Simplified source code for mecanum wheel base demo video without  
% camera and tilt
% 
% YouTube:       Teleop Taxi - HEBI Robotics
%                https://youtu.be/zaPtxre4tFc
%
% Features:      joystick input to control motions of a 4 wheel base
%
% Requirements:  MATLAB 2013b or higher
%                Logitech Gamepad F310
%                VR toolbox or MatlabInput (Github)
%
% Author:        Florian Enner
% Date:          28 March, 2016
% Last Update:   04 July, 2017
% API:           hebi-matlab-1.0
%
% Copyright 2016 HEBI Robotics

%% Setup
% select actuators for the wheels (requires names to be set appropriately)
wheels = HebiLookup.newGroupFromNames('Rosie', {
    'W1_frontLeft'
    'W2_backLeft'
    'W3_frontRight'
    'W4_backRight'
    });
 
%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    % Setup Mobile I/O Group %
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    phoneFamily = 'HEBI';
    phoneName = 'mobileIO';

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
    
    % Get the initial feedback objects that we'll reuse later
    fbkPhoneIO = phoneGroup.getNextFeedbackIO();
    latestPhoneIO = fbkPhoneIO;
    
    fbkPhoneMobile = phoneGroup.getNextFeedbackMobile();

%% Control
cmd = CommandStruct();
cmd.position = nan(1,4);
cmd.effort = nan(1,4);
maxSpeed = 6; % rad/s

while true
    
    % Read joystick axes
    tempFbk = phoneGroup.getNextFeedback( fbkPhoneIO, fbkPhoneMobile, ...
                                                  'timeout', 0 );                                   
    if ~isempty(tempFbk)
        latestPhoneMobile = fbkPhoneMobile;
        latestPhoneIO = fbkPhoneIO;
    end
    
    % Map joystick inputs to motions
    normVel = ...
        [1 1 1 1] * -latestPhoneIO.a2 + ... % forward (spin all same way)
        [-1 -1 1 1] *  latestPhoneIO.a7 + ... % rotation (spin sides opposite ways)
        [-1 1 1 -1] *  latestPhoneIO.a1;      % sideways strafe
        
    % Limit the summed velocities to the maximum 
    maxVel = max(abs(normVel));
    if maxVel > 1
       normVel = normVel / maxVel; 
    end
    
    % Account for the physical mounting of the wheels on the
    % robot, i.e., convert to local frames and send out
    mounting = [-1 -1 1 1];
    cmd.velocity = normVel .* mounting * maxSpeed;
    wheels.send(cmd);
    
    % Keep network from overloading by adding a small pause to
    % prevent busy-spinning. This is only necessary because this
    % demo does not rely on any sensor feedback.
    pause(0.001);
    
end
