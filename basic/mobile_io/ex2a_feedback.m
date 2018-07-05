% Get feedback from a module and plot it live.
%
% Assumes that you have a group created with at least 1 module in it.
%
% HEBI Robotics
% June 2018

clear *;
close all;

HebiLookup.initialize();

% Use Scope to change select a module and change the name and family to
% match the names below.  Following examples will use the same names.
familyName = 'My Family';
moduleNames = 'Test Mobile'; 
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

exampleDuration = 10; % sec
exampleTimer = tic;

group.startLog( 'dir', 'logs' );  % Starts logging in the background

disp('  Plotting gyro data from the mobile device IMU.');
disp('  Move it around to make the feedback interesting...');  

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

log = group.stopLog();  % Stops background logging

% Plot the logged position feedback
figure(101);
plot( log.time, log.gyroX );
hold on;
plot( log.time, log.gyroY );
plot( log.time, log.gyroZ );
hold off;
title('Position');
xlabel('time (sec)');
ylabel('angular velocity (rad/sec)');
legend gyroX gyroY gyroZ;
grid on;

