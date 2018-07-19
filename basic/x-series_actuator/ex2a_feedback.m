% This example does a live visualization of sensor data gathered from
% an actuator's gyroscope.
%
% For more information type:
%    help HebiGroup
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

%% Visualize Gyro Feedback
disp('  Plotting gyro data from the module IMU.');
disp('  Move the module around to make the feedback interesting...');  

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
    title('Actuator Gyro Feedback');
    xlabel('Axis');
    ylabel('Angular Velocity (rad/sec)');
    grid on;
    drawnow;

end

disp('  All Done!');
