% Make a group consisting of a single module.  A 'module' can be an
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

% Use Scope to change select a module and change the name and family to
% match the names below.  Following examples will use the same names.
familyName = 'HEBI';
moduleNames = 'mobileIO';

group = HebiLookup.newGroupFromNames( familyName, moduleNames )

