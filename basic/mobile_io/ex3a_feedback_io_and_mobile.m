% Simultaneously read analog, digital inputs, and gyro feedback,
% and visualize online.
%
% This script assumes that you have run 'startup.m' in this folder.
%
% HEBI Robotics
% July 2018

%% Setup
clear *;
close all;
HebiLookup.initialize();
group = HebiLookup.newGroupFromNames( 'HEBI', 'Mobile IO' );

%% Gather data w/ visualization
disp('  Drag the sliders on the app screen and move the device...');

% Orientation visualization
frameDisplay = FrameDisplay();
figure(2);

% create structs that we can reuse later
ioFbk = group.getNextFeedbackIO();
mobileFbk = group.getNextFeedbackMobile();

t0 = tic();
while toc(t0) < 30
    
    % Read sensor data into existing structs. This way both the data
    % is guaranteed to correspond to the same timestep.
    group.getNextFeedback(ioFbk, mobileFbk);
    
    % Visualize digital (buttons) and analog (sliders) feedback
    subplot(2,1,1);
    bar( [ioFbk.b1 ioFbk.b2 ioFbk.b3 ioFbk.b4 ioFbk.b5 ioFbk.b6 ioFbk.b7 ioFbk.b8], 'r' );
    hold on;
    bar( [ioFbk.a1 ioFbk.a2 ioFbk.a3 ioFbk.a4 ioFbk.a5 ioFbk.a6 ioFbk.a7 ioFbk.a8], 'b' );
    hold off;
    
    ylim([-1 1]);
    title('Digital Inputs (red) and Analog Inputs (blue)');
    ylabel('[-1 to 1]');
    grid on;
    
    % Visualize gyroscope data
    subplot(2,1,2);
    bar( [mobileFbk.gyroX mobileFbk.gyroY mobileFbk.gyroZ] );
    
    yAxisMaxLim = 15;
    yAxisMinLim = -15;
    ylim([yAxisMinLim yAxisMaxLim]);
    title( 'Module Gyro Feedback' );
    xlabel( 'Axis' );
    ylabel( 'Angular Velocity (rad/sec)');
    grid on;
    drawnow;
    
end

disp('  All done!');