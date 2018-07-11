% Save and load gains using the cross-API gains XML format.
%
% For more information type:
%    help GainStruct
%    help HebiUtils
%    help HebiGroup
%
% This script assumes that you have a group created with 1 module in it.
%
% HEBI Robotics
% July 2018

clear *;
close all;

HebiLookup.initialize();

familyName = 'My Family';
moduleNames = 'Test Module';  
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

% Load the gains from a saved file
newGains = HebiUtils.loadGains( '/gains/exampleGains.xml' );

% Print them to the screen
disp(newGains);

% Send the new gains to your module
group.send( 'gains', newGains );
