% Source code for X5 teaser video
% 
% YouTube:       X-Series Industrial Smart Actuator - HEBI Robotics
%                https://youtu.be/oHAddCWBobs?t=1m19s
%                
% Requirements:  MATLAB 2013b or higher
%
% Author:        Florian Enner
% Date:          23 March, 2016
% Last Update:   04 July, 2017
% API:           hebi-matlab-1.0
%
% Copyright 2016 HEBI Robotics

%% Setup
group = HebiLookup.newGroupFromNames('Demo3', 'X5-4_003');

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Demo 3) c) Torque Reaction Ping Pong
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cmd = CommandStruct();
fbk = group.getNextFeedback();
desiredPosition = fbk.position; % rad
lastTime = fbk.time; % s
desiredVelocity = -3.5;  % rad / s
coolDown = tic;
coolDownTime = 0.5; % s
torqueThreshold = 2; % Nm

while true
    
    % Read sensor feedback from the actuator
    fbk = group.getNextFeedback(fbk);
    
    % Reverse velocity direction on impact detection
    torqueErr = fbk.torque - fbk.torqueCmd;
    if torqueErr * sign(fbk.velocityCmd) > torqueThreshold 
        
        % Ignore triggers for the cooldown period 
        % to prevent oscillations
        if toc(coolDown) > coolDownTime
            desiredVelocity = -desiredVelocity;
            coolDown = tic;
        end
        
    end
    
    % Forward integrate velocity to get the desired position
    dt = (fbk.time - lastTime);
    lastTime = fbk.time;
    desiredPosition = desiredPosition + desiredVelocity * dt;
    
    % Compensate for gravity by feeding forward torques
    gravCompTorque = 5 * cos(fbk.position + pi/2);
    
    % Update joint commands
    cmd.position = desiredPosition;
    cmd.velocity = desiredVelocity;
    cmd.torque = gravCompTorque;
    group.send(cmd);
    
end