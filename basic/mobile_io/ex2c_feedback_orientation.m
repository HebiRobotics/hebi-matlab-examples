% Get orientation feedback from a mobile device, visualize online, log in 
% the background, and plot logged data offline.
%
% Assumes that you have a group created with at least 1 module in it.
%
% HEBI Robotics
% July 2018

clear *;
close all;

HebiLookup.initialize();

% Use Scope to change select a module and change the name and family to
% match the names below.  Following examples will use the same names.
familyName = 'My Family';
moduleNames = 'Test Mobile';  
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

exampleDuration = 60; % sec
exampleTimer = tic;

group.startLog( 'dir', 'logs' );  % Starts logging in the background

disp('  Visualizing orientation estimate from the mobile device.');
disp('  Move it around to make the feedback interesting...');  

fbk = group.getNextFeedbackMobile();

% Setup helper function to visualize the orientation
mobilePose = FrameDisplay();
frames = eye(4);

figure(1);
clf;

while toc(exampleTimer) < exampleDuration
    
    fbk = group.getNextFeedbackMobile();

    % Get the orientation feedback and convert to rotation matrix so that it
    % can be visualized.
    mobileQuaterion = [ fbk.orientationW ...
                        fbk.orientationX ...
                        fbk.orientationY ...
                        fbk.orientationZ ];
    mobileRotMat = HebiUtils.quat2rotMat( mobileQuaterion );

    frames(1:3,1:3) = mobileRotMat;

    mobilePose.setFrames(frames) 
    drawnow;
   
end

disp('  All done!');

log = group.stopLog();  % Stops background logging

% Plot the filtered acceleration feedback.  
%
% Note that this acceleration is different from the raw accelerometer data 
% that is returned in the regular feedback.  The mobile device is running 
% an onboard state estimator that attempts to separate out linear and 
% gravitional accelerations.
figure(101);
plot( log.time, log.accelX );
hold on;
plot( log.time, log.accelY );
plot( log.time, log.accelZ );
hold off;
title('Estimated Linear Acceleration');
xlabel('time (sec)');
ylabel('acceleration (m/sec^2)');
legend X Y Z;
grid on;



