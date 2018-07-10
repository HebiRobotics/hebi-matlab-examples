% Send velocity commands, log in the background, and plot offline.
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

cmd = CommandStruct(); % The commmand struct will have fields for position,
                       % velocity, and effort.  Fields that are empty [] 
                       % or NaN will be ignored when sending.

exampleDuration = 10; % sec
exampleTimer = tic;

group.startLog( 'dir', 'logs' ); % Starts logging in the background

% Parameters for sin/cos function
freqHz = 0.5; % Hz
freq = freqHz * 2*pi;
amp = 1.0; % rad / sec

while toc(exampleTimer) < exampleDuration
    
    fbk = group.getNextFeedback();  % Even though we don't use the feedback,
                                   % getting feedback conveniently limits
                                   % the loop rate to the feedback freq.

    cmd.velocity = amp * sin( freq * toc(exampleTimer) ); 

    group.send(cmd);
   
end

log = group.stopLog();  % Stops background logging

% Plot using some handy helper functions
HebiUtils.plotLogs( log, 'velocity' );
