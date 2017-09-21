%% HebiGroup - Command
% This example shows howto use the HebiGroup API to send basic commands 
% and to create a closed loop virtual spring
%
%%
%
% <html>
% <table border=0>
%   <tr><td>Created</td><td>July 13, 2017</td></tr>
%   <tr><td>Last Update</td><td>Sept 21, 2017</td></tr>
%   <tr><td>API Version</td><td>hebi-matlab-1.0</td></tr>
%   <tr><td>Requirements</td><td>MATLAB 2013b or higher</td></tr>
% </table>
% </html>
%
% Copyright 2017 HEBI Robotics

%% Discovery (can be omitted)
% First, we need to discover what devices are available on the network

% Show devices on the network
disp(HebiLookup);

%% Select devices and form a HebiGroup
% After discovery we can create a group that allows us to communicate with
% selected modules. Note that the parameter needs to be changed to target
% devices that are on your network.

% Select a device 
family = '*'; % any family
name = 'X-00148'; % <- modify to match the name of a device on your network!
group = HebiLookup.newGroupFromNames(family, name); 
disp(group);

%% Command zero-effort (open-loop)
% The below example keeps the module in zero-effort (on rotary modules 
% zero-torque) mode in order to be actively backdriveable.
%
% * The command needs to be contiuously re-sent to prevent the 
%   command lifetime from timing out.
% * The pause limits the rate and prevents busy spinning. This is not
%   necessary for closed-loop control as the loop is already limited by
%   the feedback rate.
cmd = CommandStruct();
t0 = tic();
while toc(t0) < 5
   cmd.effort = 0;
   group.send(cmd);
   pause(0.01);
end

%% Virtual Spring (closed-loop)
% This example shows a closed-loop controller that implements a 
% virtual spring that controls effort (torque) to drive the output 
% towards the origin.
stiffness = 1; % Nm / rad
cmd = CommandStruct();
t0 = tic();
while toc(t0) < 5
    fbk = group.getNextFeedback();
    cmd.effort = -stiffness * fbk.position; % Hooke's law: F = -k * x
    group.send(cmd);
end

%% Sine Wave (open-loop)
% This example shows an open-loop controller commanding a 1 Hz 
% sine wave using position control.
amplitude = 1;
w = 2 * pi;

cmd = CommandStruct();
t0 = tic();
t = 0;
while t < 5
    t = toc(t0);
    cmd.position = amplitude * sin( w * t );
    group.send(cmd);
    pause(0.001);
end

%% Step Input (open-loop)
% This example shows an open-loop controller commanding alternating
% step inputs of +/- amplitude to position
amplitude = 1; % [rad]
duration = 2; % [s]

cmd = CommandStruct();
direction = 1;
tNext = duration;
t0 = tic();
while toc(t0) < 5
    
    % Flip direction
    if toc(t0) >= tNext
        tNext = tNext + duration;
        direction = -direction;
    end
    
    % Command position
    cmd.position = amplitude * direction;
    group.send(cmd);
    pause(0.01);
    
end

