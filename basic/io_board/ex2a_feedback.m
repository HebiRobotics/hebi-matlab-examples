% Get feedback from a module and plot it live.
%
% Assumes that you have a group created with at least 1 module in it.
%
% HEBI Robotics
% July 2018

%% Setup
clear *;
close all;
HebiLookup.initialize();

% Use Scope to change select a module and change the name and family to
% match the names below.  Following examples will use the same names.
familyName = 'Test Family';
moduleNames = 'Test IO Board'; 
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

%% Visualize Gyro Feedback
disp('  Plotting gyro data from the mobile device IMU.');
disp('  Move it around to make the feedback interesting...');  

% Starts logging in the background
group.startLog( 'dir', 'logs' );  

figure(1);
clf;

duration = 10; % [sec]
timer = tic();
while toc(timer) < duration
    
   % read a struct of sensor data
   fbk = group.getNextFeedback();
   
   % visualize gyroscope data
   bar( [fbk.gyroX fbk.gyroY fbk.gyroZ] );
   
   % format plot
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

% Stops background logging
log = group.stopLog(); 

%% Plot the logged gyro feedback
figure(101);
plot( log.time, log.gyroX );
hold on;
plot( log.time, log.gyroY );
plot( log.time, log.gyroZ );
hold off;
title('3-Axis Gyro');
xlabel('time (sec)');
ylabel('angular velocity (rad/sec)');
legend gyroX gyroY gyroZ;
grid on;
