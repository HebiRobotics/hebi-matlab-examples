% Make a group consisting of multiple modules.
%
% Assumes that you have run 'startup.m' in this folder.
%
% HEBI Robotics
% Jun 2018

HebiLookup.initialize(); % Only needed once per session, but it doesn't
                         % hurt to do this every time we run a new script,
                         % just in case something changed on the network.

familyName = 'My Family';
moduleNames = {'Module1','Module2','Module3'};

group = HebiLookup.newGroupFromNames( familyName, moduleNames )




