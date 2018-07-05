% Send velocity commands based on feedback from a module's internal gyro, 
% log in the background, and plot offline.
%
% Assumes that you have a group created with 1 module in it.
%
% HEBI Robotics
% Jun 2018

HebiLookup.initialize();

% Use Scope to change select a module and change the name and family to
% match the names below.  Following examples will use the same names.
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
HebiUtils.plotLogs( log, 'velocity', 101 );
HebiUtils.plotLogs( log, 'gyroZ', 102 );
