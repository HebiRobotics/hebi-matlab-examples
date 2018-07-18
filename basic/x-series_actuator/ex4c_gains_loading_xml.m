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

% Load the gains from a saved file
gainsFileFolder = 'gains/';
gainsFileName = 'exampleGains.xml';     % the '.gains' is optional
newGains = HebiUtils.loadGains( [gainsFileFolder gainsFileName] );

% Print them to the screen
disp(newGains);

% Send the new gains to your module
group.send( 'gains', newGains );
