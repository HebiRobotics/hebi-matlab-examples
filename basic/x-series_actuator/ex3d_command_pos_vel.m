% Send simultaneous position and velocity commands, log in the background, 
% and plot offline.
%
% For more information type:
%    help CommandStruct
%    help HebiGroup
%
% This script assumes you can create a group with 1 module.
%
% HEBI Robotics
% June 2018

%%
clear *;
close all;

HebiLookup.initialize();

familyName = 'My Family';
moduleNames = 'Test Module';  
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

cmd = CommandStruct();

exampleDuration = 10; % [sec]
exampleTimer = tic;

group.startLog( 'dir', 'logs' );

% Parameters for sin/cos function
freqHz = 1.0;           % [Hz]
freq = freqHz * 2*pi;   % [rad / sec]
amp = deg2rad( 45 );    % [rad]

while toc(exampleTimer) < exampleDuration

    % Even though we don't use the feedback, getting feedback conveniently 
    % limits the loop rate to the feedback frequency 
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
