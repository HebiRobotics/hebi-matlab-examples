% Send effort commands, log in the background, and plot offline.
%
% For more information type:
%    help CommandStruct
%    help HebiGroup
%
% This script assumes that you have a group created with 1 module in it.
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

% The commmand struct will have fields for position, velocity, and effort.  
% Fields that are empty [] or NaN will be ignored when sending.
cmd = CommandStruct(); 
                       

exampleDuration = 10; % [sec]
exampleTimer = tic;

% Starts logging in the background
group.startLog( 'dir', 'logs' ); 

% Parameters for sin/cos function
freqHz = 0.5;           % [Hz]
freq = freqHz * 2*pi;   % [rad / sec]
amp = 1;                % [Nm]

while toc(exampleTimer) < exampleDuration
    
    % Even though we don't use the feedback, getting feedback conveniently 
    % limits the loop rate to the feedback frequency                 
    fbk = group.getNextFeedback();  
                                  
    cmd.effort = amp * sin( freq * toc(exampleTimer) ); 

    group.send(cmd);
   
end

% Stops background logging
log = group.stopLog();

% Plot using some handy helper functions
HebiUtils.plotLogs( log, 'effort' );
