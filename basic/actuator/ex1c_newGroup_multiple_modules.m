% Make a group consisting of multiple modules.  A 'module' can be an
% actuator, an I/O board, or a mobile device running the Mobile I/O app.
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
familyName = 'Arm';
moduleNames = {'J4_wrist1','J5_wrist2','J6_wrist3'};
% familyName = 'Test Family';
% moduleNames = {'Actuator1','Actuator2','Actuator3'};

group = HebiLookup.newGroupFromNames( familyName, moduleNames )
