% This example shows howto use the HebiGroup API to access basic feedback
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
% We first need to discover what devices are available on the network
disp(HebiLookup);

%% Select devices and form a HebiGroup
% After discovery we can create a group that allows us to communicate with
% selected modules. Note that the parameter needs to be changed to target
% devices that are on your network.
serials = 'X-80040'; % <- modify to match a device on your network!
group = HebiLookup.newGroupFromSerialNumbers(serials); 

%% Access feedback
% Feedback is in SI units and can be accessed as below. If a group contains
% more than one module, the returned feedback contains vectors instead of
% scalars.
fbk = group.getNextFeedback();
display(fbk);

%% Output position feedback once a second
group.setFeedbackFrequency(1); % reduce rate from 100 Hz default
t0 = tic();
while toc(t0) < 5
    fbk = group.getNextFeedback();
    disp(['position: ' num2str(fbk.position)]);
end