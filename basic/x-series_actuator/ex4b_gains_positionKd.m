% Change the D-gain of the position controller on a module while commanding
% a step change in position.
%
% For more information type:
%    help GainStruct
%    help HebiGroup
%
% This script assumes that you have a group created with 1 module in it.
%
% HEBI Robotics
% July 2018

clear *;
close all;

HebiLookup.initialize();

familyName = 'My Family';
moduleNames = 'Test Module';  
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

cmd = CommandStruct();
gains = GainStruct(); % Like the commmand struct, the gains struct will 
                      % have fields for every possible gain value.  Fields 
                      % that are empty [] or NaN will be ignored when
                      % sending.

exampleDuration = 4; % sec

group.startLog( 'dir', 'logs' ); 

% Parameters for step function
stepPeriod = 1.0; % sec
stepPosition = pi/8; % Rad

newPositionKpGains = [ 10  10 ];  % Make the position controller stiff
newPositionKdGains = [ 0.0 0.1 ];  % Show performance with and without 
                                   % damping.

for i = 1:length(newPositionKpGains)
    
    gains.positionKp = newPositionKpGains(i);
    gains.positionKd = newPositionKdGains(i);
    
    group.send( 'gains', gains );
    disp(['  Setting Kd to: ' num2str(newPositionKdGains(i))]);

    exampleTimer = tic;
    while toc(exampleTimer) < exampleDuration

       fbk = group.getNextFeedback();

       if rem( toc(exampleTimer), 2*stepPeriod ) > stepPeriod
           cmd.position = stepPosition;
       else
           cmd.position = 0;
       end

       group.send(cmd);

    end
end

pause(stepPeriod);

disp('  All done!');

log = group.stopLog();  % Stops background logging

% Plot using some handy helper functions
HebiUtils.plotLogs( log, 'position', 'figNum', 101 );
