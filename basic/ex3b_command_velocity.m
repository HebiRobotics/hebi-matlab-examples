% Send velocity commands, log in the background, and plot offline.
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

while toc < exampleDuration
    
   fbk = group.getNextFeedback();
   
   cmd.velocity = sin( 2*pi * toc ); % 1 Hz sine-wave sweep
   
   group.send(cmd);
   
end

log = group.stopLog();  % Stops background logging

% Plot using some handy helper functions
HebiUtils.plotLogs( log, 'velocity' );
