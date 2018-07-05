% Save and load gains using the cross-API gains XML format.
%
% Assumes that you have a group created with 1 module in it.
%
% HEBI Robotics
% July 2018

clear *;
close all;

HebiLookup.initialize();

familyName = 'My Family';
moduleNames = 'Test Module';  
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

% Get the gains that are currently active on the module and save them
gains = group.getGains();
HebiUtils.saveGains( gains, '/gains/myActuatorGains.xml' );
