% Send simultaneous position and velocity commands, log in the background, 
% and plot offline.
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

% Parameters for sin/cos function
freqHz = 1; % Hz
freq = freqHz * 2*pi;
amplitude = deg2rad(45); % rad

while toc < exampleDuration
    
   fbk = group.getNextFeedback();
   
   cmd.position = amplitude * cos( freq*toc );
   cmd.velocity = amplitude * freq * sin( freq*toc );
   
   group.send(cmd);
   
end

log = group.stopLog();  % Stops background logging

% Plot using some handy helper functions
HebiUtils.plotLogs( log, 'position' );
HebiUtils.plotLogs( log, 'velocity' );