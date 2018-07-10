% Send velocity commands based on feedback from a module's internal gyro, 
% log in the background, and plot offline.
%
% For more information type:
%    help CommandStruct
%    help HebiGroup
%
% This script assumes that you have a group created with 1 module in it.
%
% HEBI Robotics
% June 2018

clear *;
close all;

HebiLookup.initialize();

familyName = 'My Family';
moduleNames = 'Test Module';  
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

cmd = CommandStruct();

exampleDuration = 15; % sec
exampleTimer = tic;

group.startLog( 'dir', 'logs' ); 

disp('  Move the module to make the output move...');  

while toc(exampleTimer) < exampleDuration
    
    fbk = group.getNextFeedback();

    cmd.velocity = -fbk.gyroZ;  % Command a velocity that counters the  
                               % measured angular velocity around z-axis.
    group.send(cmd);
   
end

disp('  All done!');

log = group.stopLog();  % Stops background logging

% Plot using some handy helper functions
HebiUtils.plotLogs( log, 'velocity', 'figNum', 101 );
HebiUtils.plotLogs( log, 'gyroZ', 'figNum', 102 );
