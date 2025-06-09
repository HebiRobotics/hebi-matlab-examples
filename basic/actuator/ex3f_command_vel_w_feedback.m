% Send velocity commands based on feedback from a module's internal gyro, 
% log in the background, and plot offline.
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

%% Closed-Loop Controller (Velocity)
cmd = CommandStruct();
group.startLog( 'dir', 'logs' ); 

disp('  Move the module to make the output move...');  
duration = 15; % [sec]
timer = tic();
while toc(timer) < duration
    
    fbk = group.getNextFeedback();

    % Command a velocity that counters the measured angular velocity 
    % around the z-axis (same axis as the output).
    cmd.velocity = -fbk.gyroZ; 
    
    group.send(cmd);
   
end

disp('  All done!');

% Stop background logging and plot data
log = group.stopLog(); 
HebiUtils.plotLogs( log, 'velocity', 'figNum', 101 );
HebiUtils.plotLogs( log, 'gyroZ', 'figNum', 102 );
