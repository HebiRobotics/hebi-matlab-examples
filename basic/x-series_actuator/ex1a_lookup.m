% Initialize the lookup of modules and display information for any modules
% on the network. A 'module' can be an actuator, and I/O board, or a 
% mobile device running the Mobile I/O app.
%
% For more information type:
%    help HebiLookup
%
% This script assumes that you have run 'startup.m' in this folder.
%
% HEBI Robotics
% June 2018

%%
clear *;
close all;

HebiLookup.initialize();

disp(HebiLookup);  

disp(' NOTE:');
disp('  The table above should show the information for all the modules');
disp('  on the local network.  If this table is empty make sure that the');
disp('  modules are connected, powered on, and that the status LEDs are'); 
disp('  displaying a green soft-fade.');
disp('  ');
