% Source code for mecanum wheel base demo video
% 
% YouTube:       Teleop Taxi - HEBI Robotics
%                https://youtu.be/zaPtxre4tFc
%
% Features:      joystick input to control motions of a 4 wheel base
%                joystick input to control camera tilt
%                displays live video streaming
%
% Requirements:  MATLAB 2013b or higher
%                Logitech Gamepad F310
%                VR toolbox or MatlabInput (Github)
%                HebiCam
%
% Author:        Florian Enner
% Date:          28 March, 2016
% Last Update:   04 July, 2017
% API:           hebi-matlab-1.0
%
% Copyright 2016 HEBI Robotics

%% Setup
% select actuators for the wheels (requires names to be set appropriately)
wheels = HebiLookup.newGroupFromNames('Cart', {
    'left_front'
    'left_back'
    'right_front'
    'right_back'
    });
% select actuator for camera tilt
tilt = HebiLookup.newGroupFromNames('Cart','camera_tilt');
cam = HebiCam('http://10.10.10.233:8080/video');
joy = vrjoystick(1); % alternatively get HebiJoystick(1)

%% Control
cmd = CommandStruct();
tiltSpeed = 2; % rad/s
maxSpeed = 10; % rad/s
I = imshow(cam.getsnapshot);

while true
    
    % Read joystick axes
    [axes, buttons, povs] = read(joy);
    deadZone = abs(axes) < 0.1;
    axes(deadZone) = 0;
    
    % Map joystick inputs to motions
    normVel = ...
        [1  1  1  1] * -axes(5) + ... % forward (spin all same way)
        [1  1 -1 -1] *  axes(4) + ... % rotation (spin sides opposite ways)
        [1 -1 -1  1] *  axes(1);      % sideways strafe
        
    % Limit the summed velocities to the maximum 
    maxVel = max(abs(normVel));
    if maxVel > 1
       normVel = normVel / maxVel; 
    end
    
    % Account for the physical mounting of the wheels on the
    % robot, i.e., convert to local frames and send out
    mounting = [-1  1  1 -1];
    cmd.velocity = normVel .* mounting * maxSpeed;
    wheels.send(cmd);
    
    % Map joystick input to camera tilt
    cmd.velocity = tiltSpeed * axes(2);
    tilt.send(cmd);
    
    % Display live image from phone
    I.CData = getsnapshot(cam);
    drawnow;
    
    % The loop is already limited by the camera frame rate, so
    % there does not need to be a pause to prevent busy spinning.
    
end
