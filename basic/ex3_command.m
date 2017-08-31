%% HebiGroup - Command
% This example shows howto use the HebiGroup API to send basic commands 
% and to create a closed loop virtual spring
%
%%
%
% <html>
% <table border=0>
%   <tr><td>Created</td><td>July 13, 2017</td></tr>
%   <tr><td>Last Update</td><td>Aug 22, 2017</td></tr>
%   <tr><td>API Version</td><td>hebi-matlab-1.0-rc2</td></tr>
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
% sine wave using simultaneous position and velocity control.
cmd = CommandStruct();
w = 2 * pi;
t0 = tic();
t = 0;
while t < 5
    t = toc(t0);
    cmd.position = sin( w * t );
    cmd.velocity = cos( w * t );
    group.send(cmd);
    pause(0.001);
end
