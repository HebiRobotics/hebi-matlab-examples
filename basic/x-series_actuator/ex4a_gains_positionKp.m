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

%% Setup
clear *;
close all;
HebiLookup.initialize();

familyName = 'Test Family';
moduleNames = 'Test Actuator'; 
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

%% Position Step Input w/ different Kp gains
% Like the commmand struct, the gains struct has fields for every 
% possible gain value.  Fields that are empty [] or NaN will be ignored 
% when sending.
gains = GainStruct(); 
cmd = CommandStruct();
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
    duration = 4; % [sec]
    timer = tic;
    while toc(timer) < duration
        
        % Get feedback to limit the loop rate
        fbk = group.getNextFeedback();
        
        % Calculate position step function and send command
        if rem( toc(timer), 2*stepPeriod ) > stepPeriod
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

% Stop background logging and plot data using helper functions
log = group.stopLog();  
HebiUtils.plotLogs( log, 'position', 'figNum', 101 );
