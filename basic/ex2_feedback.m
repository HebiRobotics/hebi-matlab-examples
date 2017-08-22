%% HebiGroup - Feedback
% This example shows howto use the HebiGroup API to access basic feedback
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
name = 'Elbow'; % <- modify to match the name of a device on your network!
group = HebiLookup.newGroupFromNames(family, name); 
disp(group);

%% Access feedback
% Feedback is in SI units and can be accessed as below. If a group contains
% more than one module, the returned feedback contains vectors instead of
% scalars.

% Display the most recent feedback
fbk = group.getNextFeedback();
display(fbk);

%% Continuously access feedback

% Display position feedback once a second for 5 seconds
group.setFeedbackFrequency(1); % reduce rate from 100 Hz default
t0 = tic();
while toc(t0) < 5
    fbk = group.getNextFeedback();
    disp(['position: ' num2str(fbk.position)]);
end