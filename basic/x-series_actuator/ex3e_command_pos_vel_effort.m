% Send simultaneous position/velocity/effort commands, log in the 
% background, and plot offline.
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

exampleDuration = 20; % sec
exampleTimer = tic;

group.startLog();  % Starts logging in the background

% Parameters for sin/cos function
freqHz = 1; % Hz
freq = freqHz * 2*pi;
amplitude = deg2rad(45); % rad

% Inertia parameters for converting acceleration to torque
inertia = 1E6; % kg * m^2

while toc(exampleTimer) < exampleDuration
    
   fbk = group.getNextFeedback();
   
   cmd.position = amplitude * cos( freq*toc );
   cmd.velocity = freq * amplitude * sin( freq*toc );
   cmd.effort = inertia * freq^2 * amplitude * sin( freq*toc );
   
   group.send(cmd);
   
end

log = group.stopLog();  % Stops background logging

% Plot using some handy helper functions
HebiUtils.plotLogs( log, 'position' );
HebiUtils.plotLogs( log, 'velocity' );
HebiUtils.plotLogs( log, 'effort' );
