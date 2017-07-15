% Source code for X5 teaser video
% 
% YouTube:       X-Series Industrial Smart Actuator - HEBI Robotics
%                https://youtu.be/oHAddCWBobs?t=30s
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
group = HebiLookup.newGroupFromFamily('Demo2');

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Demo 2) Keep output stable by cancelling IMU
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cmd = CommandStruct();
fbk = group.getNextFeedback();
while true
    
    % Read feedback and counter IMU
    fbk = group.getNextFeedback(fbk); % call reuses memory
    cmd.velocity = -fbk.gyroZ; % gyro feedback is already unbiased
    group.send(cmd);
    
end