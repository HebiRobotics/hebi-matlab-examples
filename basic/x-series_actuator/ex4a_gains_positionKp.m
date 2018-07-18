% Change the P-gain of the position controller on a module while commanding
% a step change in position.
%
% For more information type:
%    help GainStruct
%    help HebiGroup
%
% This script assumes you can create a group with 1 module.
%
% HEBI Robotics
% July 2018

%%
clear *;
close all;

HebiLookup.initialize();

familyName = 'My Family';
moduleNames = 'Test Module';  
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

% Like the commmand struct, the gains struct will have fields for every 
% possible gain value.  Fields that are empty [] or NaN will be ignored 
% when sending.
cmd = CommandStruct();
gains = GainStruct(); 

exampleDuration = 4;    % [sec]

group.startLog( 'dir', 'logs' ); 

% Parameters for step function
stepPeriod = 1.0;       % [sec]
stepPosition = pi/8;    % [rad]

% Make the position controller gradually stiffer. 
newPositionKpGains = [ 0.2 0.5 10 ]; 

for i = 1:length(newPositionKpGains)
    
    % Update gains struct and send to the actuator
    gains.positionKp = newPositionKpGains(i);
    group.send( 'gains', gains );
    disp(['  Setting Kp to: ' num2str(newPositionKpGains(i))]);

    % Command a step function to show actuator response
    exampleTimer = tic;
    while toc(exampleTimer) < exampleDuration
        
        % Get feedback to limit the loop rate
        fbk = group.getNextFeedback();
        
        % Calculate position step function and send command
        if rem( toc(exampleTimer), 2*stepPeriod ) > stepPeriod
           cmd.position = stepPosition;
        else
           cmd.position = 0;
        end
        group.send(cmd);

    end
end

% Pause to let the actuator response to settle
pause(stepPeriod);

disp('  All done!');

% Stops background logging
log = group.stopLog();  

% Plot using some handy helper functions
HebiUtils.plotLogs( log, 'position', 'figNum', 101 );
