% Get feedback from a module and plot it live.
%
% This script assumes that you have run 'startup.m' in this folder.
%
% HEBI Robotics
% July 2018

%% Setup
clear *;
close all;
HebiLookup.initialize();

% Use Scope to change select a module and change the name and family to
% match the names below.  Following examples will use the same names.
familyName = 'HEBI';
deviceName = 'mobileIO';

% The HebiMobileIO utility wrapper makes it easier to work with the
% relevant parts of the group API.
mobileIO = HebiMobileIO.findDevice(familyName, deviceName);
mobileIO.setDefaults();

%% Visualize Gyro Feedback
disp('  Plotting gyro data from the mobile device IMU.');
disp('  Move it around to make the feedback interesting...');  

figure(1);
clf;

duration = 20; % [sec]
timer = tic();
while toc(timer) < duration

    % Get Feedback
    mobileIO.update();  
    fbkMobile = mobileIO.getFeedbackMobile();

    % Visualize gyroscope data
    bar( [fbkMobile.gyroX fbkMobile.gyroY fbkMobile.gyroZ] );

    % Format plot
    yAxisMaxLim = 15; 
    yAxisMinLim = -15;
    ylim([yAxisMinLim yAxisMaxLim]);
    title( 'Module Gyro Feedback' );
    xlabel( 'Axis' );
    ylabel( 'Angular Velocity (rad/sec)');
    grid on;
    drawnow;

end