% Make a group consisting of multiple modules.  A 'module' can be an
% actuator, and I/O board, or a mobile device running the Mobile I/O app.
%
% For more information type:
%    help HebiGroup
%    help HebiLookup    
%
% This script assumes that you have run 'startup.m' in this folder.
%
% HEBI Robotics
% June 2018

%%
clear *;
close all;

% Only needed once per session, but it doesn't hurt to do this every time
% we run a new script, just in case something changed on the network.
HebiLookup.initialize();

familyName = 'Test Family';
moduleNames = {'Module1','Module2','Module3'};

group = HebiLookup.newGroupFromNames( familyName, moduleNames )
