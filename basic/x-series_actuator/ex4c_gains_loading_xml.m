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

familyName = 'Test Family';
moduleNames = 'Test Actuator';  
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

% Load the gains from a saved file
newGains = HebiUtils.loadGains( './gains/exampleGains.xml' );
disp(newGains); % display

% Send the new gains to the actuator
group.send( 'gains', newGains );
