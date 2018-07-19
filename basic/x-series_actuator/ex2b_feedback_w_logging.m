% Get feedback from a module, log in the background, and plot offline.
%
% For more information type:
%    help HebiGroup
%
% This script assumes you can create a group with at least 1 module.
%
% HEBI Robotics
% June 2018

%% Setup group
clear *;
close all;
HebiLookup.initialize();

% Use Scope to change select a module and change the name and family to
% match the names below.  Following examples will use the same names.
familyName = 'Test Family';
moduleNames = 'Test Actuator';  
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

%% Live Visualization
% Starts logging in the background. Note that logging can be enabled at any
% time, and that it does not negatively affect the performance of MATLAB.
group.startLog( 'dir', 'logs' );

disp('  Use Scope to command the module and make it move...');
figure(1);
clf;

duration = 10; % [sec]
timer = tic();
while toc(timer) < duration

    fbk = group.getNextFeedback();
    
    bar( fbk.velocity );
    
    yAxisMaxLim = 5; 
    yAxisMinLim = -5;
    ylim([yAxisMinLim yAxisMaxLim]);
    title( 'Module Output Velocity' );
    ylabel( 'Angular Velocity (rad/sec)');
    grid on;
    drawnow;

end

disp('  All done!');

% Stops background logging and converts the logged data into MATLAB
% matrices that can be easily plotted.
log = group.stopLog();  

%% Offline Visualization
% Plot the logged position feedback
figure(101);
plot(log.time, log.position);
title('Position');
xlabel('time (sec)');
ylabel('position (rad)');
grid on;

% Plot the logged velocity feedback
figure(102);
plot(log.time, log.velocity);
title('Velocity');
xlabel('time (sec)');
ylabel('velocity (rad/sec)');
grid on;


