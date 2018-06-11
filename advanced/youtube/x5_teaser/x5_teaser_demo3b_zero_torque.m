% Source code for X5 teaser video
% 
% YouTube:       X-Series Industrial Smart Actuator - HEBI Robotics
%                https://youtu.be/oHAddCWBobs?t=1m9s
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
% Demo 3) b) Zero Torque Mode
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cmd = CommandStruct();
cmd.torque = 0;

% Note that the zero torque commands needs to be sent in a loop to  
% prevent the command lifetime from turning off the actuator. While the
% command lifetime could be deactivated for this particular demo, we prefer
% to always keep it enabled. Aside from the safety benefits it also
% prevents anyone else from commanding the actuator during the demo.
while true
    
    % send command to actuator
    group.send(cmd);
    
    % pause some time to keep this loop from busy-spinning
    % and overloading the network
    pause(0.01);
    
end