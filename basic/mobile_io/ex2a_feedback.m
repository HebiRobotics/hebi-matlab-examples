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
deviceName = 'Mobile IO';

% Loop to keep trying to form the arm group.  Make sure the mobile device
% has the correct name / family to match the ones above and that it is on
% the same network as this computer.  You can check for this using Scope.
while true  
    try
        fprintf('Searching for phone Controller...\n');
        group = HebiLookup.newGroupFromNames( ...
                        familyName, deviceName );        
        disp('Phone Found.  Starting up');
        break;
    catch
        % If we failed to make a group, pause a bit before trying again.
        pause(1.0);
    end
end

mobileIO = HebiMobileIO( group );
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