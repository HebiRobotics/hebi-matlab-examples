% Get feedback from a module and plot it live.
%
% For more information type:
%    help HebiGroup
%
% This script assumes you have a group created with at least 1 module.
%
% HEBI Robotics
% June 2018

clear *;
close all;

HebiLookup.initialize();

% Use Scope to change select a module and change the name and family to
% match the names below.  Following examples will use the same names.
familyName = 'My Family';
moduleNames = 'Test Module';  
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

exampleDuration = 10; % sec
exampleTimer = tic;

disp('  Plotting gyro data from the module IMU.');
disp('  Move the module around to make the feedback interesting...');  

% Flag so that we only set the title, etc of plot one time
isFirstDraw = true;

figure(1);
clf;

while toc(exampleTimer) < exampleDuration

    fbk = group.getNextFeedback();

    bar( [fbk.gyroX fbk.gyroY fbk.gyroZ] );

    yAxisMaxLim = 15; 
    yAxisMinLim = -15;
    ylim([yAxisMinLim yAxisMaxLim]);

    title( 'Module Gyro Feedback' );
    xlabel( 'Axis' );
    ylabel( 'Angular Velocity (rad/sec)');
    grid on;

    drawnow;

end

disp('  All Done!');
