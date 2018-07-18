% Send effort commands, log in the background, and plot offline.
%
% For more information type:
%    help CommandStruct
%    help HebiGroup
%
% This script assumes you can create a group with 1 module.
%
% HEBI Robotics
% June 2018

%% Setup
clear *;
close all;
HebiLookup.initialize();

familyName = 'Test Family';
moduleNames = 'Test Actuator'; 
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

%% Command Loop (Effort)
% The commmand struct has fields for position, velocity, and effort.  
% Fields that are empty [] or NaN will be ignored when sending.
cmd = CommandStruct(); 
                       
% Starts logging in the background
group.startLog( 'dir', 'logs' ); 

% Parameters for sin/cos function
freqHz = 0.5;           % [Hz]
freq = freqHz * 2*pi;   % [rad / sec]
amp = 1;                % [Nm]

duration = 10; % [sec]
timer = tic();
while toc(timer) < duration
    
    % Even though we don't use the feedback, getting feedback conveniently 
    % limits the loop rate to the feedback frequency                 
    fbk = group.getNextFeedback();  
                
    % Update effort set point
    cmd.effort = amp * sin( freq * toc(timer) ); 
    group.send(cmd);
   
end

% Stop logging and plot the effort data using helper functions
log = group.stopLog();
HebiUtils.plotLogs( log, 'effort' );