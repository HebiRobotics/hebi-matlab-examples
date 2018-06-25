% Send velocity commands based on feedback from a module's internal gyro, 
% log in the background, and plot offline.
%
% Assumes that you have a group created with 1 module in it.
%
% HEBI Robotics
% Jun 2018

HebiLookup.initialize();

exampleDuration = 20; % sec

cmd = CommandStruct();

tic;

group.startLog();  % Starts logging in the background

disp('  Move the module to make the output move...');  

while toc < exampleDuration
    
   fbk = group.getNextFeedback();
   
   cmd.velocity = -fbk.gyroZ;  % Command a velocity that counters the  
                               % measured angular velocity around z-axis.
   group.send(cmd);
   
end

disp('  All done!');

log = group.stopLog();  % Stops background logging

% Plot using some handy helper functions
HebiUtils.plotLogs( log, 'velocity' );
