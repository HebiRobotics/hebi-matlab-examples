% Send position commands, log in the background, and plot offline.
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

exampleDuration = 10; % sec
exampleTimer = tic;

group.startLog( 'dir', 'logs' );  % Starts logging in a sub-directory

% Parameters for sin/cos function
freqHz = 1.0; % Hz
freq = freqHz * 2*pi;
amp = deg2rad( 45 ); % radians

while toc(exampleTimer) < exampleDuration
    
   fbk = group.getNextFeedback();
   
   cmd.position = amp * sin( freq * toc(exampleTimer) ); 
   
   group.send(cmd);
   
end

log = group.stopLog();  % Stops background logging

% Plot using some handy helper functions
HebiUtils.plotLogs( log, 'position' );