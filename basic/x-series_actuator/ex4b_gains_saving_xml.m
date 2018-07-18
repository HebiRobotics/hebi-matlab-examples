% Save and load gains using the cross-API gains XML format.
%
% For more information type:
%    help GainStruct
%    help HebiUtils
%    help HebiGroup
%
% This script assumes you can create a group with 1 module.
%
% HEBI Robotics
% July 2018

%%
clear *;
close all;

HebiLookup.initialize();

familyName = 'My Family';
moduleNames = 'Test Module';  
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

% Get the gains that are currently active on the module and save them
gains = group.getGains();
HebiUtils.saveGains( gains, '/gains/myActuatorGains.xml' );
