% This example shows howto use the HebiGroup API to send basic commands
% and to create a virtual spring
%
% (hint: individual sections can be run using ctrl+enter)
% 
% Requirements:  MATLAB 2013b or higher
%
% Author:        Florian Enner
% Created:       13 July, 2017
% API:           hebi-matlab-1.0
%
% Copyright 2017 HEBI Robotics

%% Discovery (can be omitted)
disp(HebiLookup);

%% Setup
serials = 'X-80040'; % Selection (modify to match your devices!)
group = HebiLookup.newGroupFromSerialNumbers(serials);
disp(group);

%% Command zero-torque
% The below example keeps the module in zero-torque mode in order to be
% actively backdriveable. 
%
% * The command needs to be contiuously re-sent to prevent the 
%   command lifetime from timing out.
% * The pause limits the rate and prevents busy spinning. This is not
%   necessary for closed-loop control as the feedback rate is already 
%   limiting.
cmd = CommandStruct();
t0 = tic();
while toc(t0) < 5
   cmd.torque = 0;
   group.send(cmd);
   pause(0.01);
end

%% Virtual Spring (closed-loop)
% This example shows a closed-loop controller that implements a 
% virtual spring that controls torque to drive the output towards the 
% origin.
stiffness = 1; % Nm / rad
cmd = CommandStruct();
t0 = tic();
while toc(t0) < 5
    fbk = group.getNextFeedback();
    cmd.torque = -stiffness * fbk.position; % Hooke's law: F = -k * x
    group.send(cmd);
end

%% Sine Wave (open-loop)
% This example shows an open-loop controller commanding a 1 Hz 
% sine wave using simultaneous position and velocity control.
cmd = CommandStruct();
w = 2 * pi;
t0 = tic();
t = 0;
while t < 5
    t = toc(t0);
    cmd.position = sin( w * t );
    cmd.velocity = cos( w * t );
    group.set(cmd);
    pause(0.001);
end
