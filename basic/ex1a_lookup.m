% Initialize the lookup of modules and display information for any modules
% on the network.
%
% Assumes that you have run 'startup.m' in this folder.
%
% HEBI Robotics
% Jun 2018

HebiLookup.initialize();

disp(HebiLookup);  

disp(' NOTE:');
disp('  The table above should show the information for all the modules');
disp('  on the local network.  If this table is empty make sure that the');
disp('  modules are connected, powered on, and that the status LEDs are'); 
disp('  displaying a green soft-fade.');
disp('  ');
