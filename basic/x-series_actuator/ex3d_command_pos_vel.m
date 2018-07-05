% Send simultaneous position and velocity commands, log in the background, 
% and plot offline.
%
% Assumes that you have a group created with 1 module in it.
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

exampleDuration = 10; % sec
exampleTimer = tic;

group.startLog( 'dir', 'logs' );

% Parameters for sin/cos function
freqHz = 1.0; % Hz
freq = freqHz * 2*pi;
amp = deg2rad( 45 ); % radians

while toc(exampleTimer) < exampleDuration
    
   fbk = group.getNextFeedback();
   
   % Position Command
   cmdPosition = amp * sin( freq*toc(exampleTimer) );
   
   % Velocity Command (time-derivate of position)
   cmdVelocity = freq * amp * cos( freq*toc(exampleTimer) );
   
   cmd.position = cmdPosition;
   cmd.velocity = cmdVelocity;
   
   group.send(cmd);
   
end

log = group.stopLog();  % Stops background logging

% Plot using some handy helper functions
HebiUtils.plotLogs( log, 'position', 'figNum', 101 );
HebiUtils.plotLogs( log, 'velocity', 'figNum', 102 );
