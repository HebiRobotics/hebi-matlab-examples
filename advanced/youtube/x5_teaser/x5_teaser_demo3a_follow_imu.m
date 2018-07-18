% Source code for X5 teaser video
% 
% YouTube:       X-Series Industrial Smart Actuator - HEBI Robotics
%                https://youtu.be/oHAddCWBobs?t=46s
%                
% Requirements:  MATLAB 2013b or higher
%                An Android phone running the HEBI Node App from the
%                Google Play Store:
%                https://play.google.com/store/apps/details?id=us.hebi.android.node&hl=en
%
% Author:        Florian Enner
% Date:          23 March, 2016
% Last Update:   04 July, 2017
% API:           hebi-matlab-1.0
%
% Copyright 2016 HEBI Robotics

%% Setup
group = HebiLookup.newGroupFromNames('Demo3', 'X5-4_003');
phone = HebiLookup.newGroupFromNames('*','HEBI_Nexus5'); % See link above 
                                                         % to download app

%% Get Gyro Bias (don't move phone!)
phone.startLog();
pause(5);
gyroBiasZ = mean(phone.stopLog().gyroZ);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Demo 3) a) Follow Phone's IMU
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cmd = CommandStruct();
phoneFbk = phone.getNextFeedback();
jointFbk = group.getNextFeedback();
desiredPosition = jointFbk.position;
lastTime = phoneFbk.time;
lastVelocity = phoneFbk.gyroZ - gyroBiasZ;
inertia = 1.2 * (15 * 0.0254)^2;

while true
    
    % Read sensors from both the phone as well as the joint
    phoneFbk = phone.getNextFeedback(phoneFbk);
    jointFbk = group.getNextFeedback(jointFbk);
    
    % Copy velocity from unbiased gyro reading 
    desiredVelocity = phoneFbk.gyroZ - gyroBiasZ;
    
    % Forward integrate velocity to get the desired position
    dt = phoneFbk.time - lastTime;
    lastTime = phoneFbk.time;
    desiredPosition = desiredPosition + desiredVelocity * dt;
    
    % Compensate gravity by feeding forward torques
    gravCompTorque =  5 * cos(fbk.position + pi/2);
    
    % Compensate joint accelerations by feeding forward torques
    jointAcceleration = desiredVelocity - lastVelocity;
    accelCompTorque = inertia * jointAcceleration;  
    lastVelocity = desiredVelocity;
    
    % Update joint commands
    cmd.position = desiredPosition;
    cmd.velocity = desiredVelocity;
    cmd.torque = accelCompTorque + gravCompTorque;
    group.send(cmd);
    
end