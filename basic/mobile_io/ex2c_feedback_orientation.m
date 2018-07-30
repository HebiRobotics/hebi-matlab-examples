% Get orientation feedback from a mobile device, visualize online, log in 
% the background, and plot logged data offline.
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
familyName = 'Dave';
moduleNames = 'IO_00060';
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

%% Visualize Device Orientation
disp('  Visualizing orientation estimate from the mobile device.');
disp('  Move it around to make the feedback interesting...');  

% Start logging in the background
group.startLog( 'dir', 'logs' ); 

% Setup helper function to visualize the orientation
xyzLimits = [ -0.05 0.05;
              -0.05 0.05;
              -0.05 0.05 ];
mobilePose = FrameDisplay( [], [], xyzLimits );
frames = eye(4);

figure(1);
clf;

duration = 60; % [sec]
timer = tic();
while toc(timer) < duration
    
    fbk = group.getNextFeedbackMobile();

    % Get the orientation feedback and convert to rotation matrix so that it
    % can be visualized.
    mobileQuaternion = [ fbk.orientationW ...
                         fbk.orientationX ...
                         fbk.orientationY ...
                         fbk.orientationZ ];
    mobileRotMat = HebiUtils.quat2rotMat( mobileQuaternion );

    frames(1:3,1:3) = mobileRotMat;

    mobilePose.setFrames(frames) 
    drawnow;
   
end

disp('  All done!');

% Stop background logging
log = group.stopLogMobile();  

%% Plot the filtered acceleration feedback. 
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



