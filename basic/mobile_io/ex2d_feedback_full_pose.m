% Get feedback from a module, log in the background, and plot offline.
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
   mobileQuaterion = [ fbk.arOrientationW ...
                       fbk.arOrientationX ...
                       fbk.arOrientationY ...
                       fbk.arOrientationZ ];
   mobileRotMat = HebiUtils.quat2rotMat( mobileQuaterion );
   mobileXYZ = [ arPositionX;
                 arPositionY;
                 arPositionZ ];
    
   frames(1:3,1:3) = mobileRotMat;
   frames(1:3,4) = mobileXYZ;
  
   mobilePose.setFrames(frames) 
   drawnow;
   
end

disp('  All done!');

log = group.stopLog();  % Stops background logging

% Plot the logged position feedback
figure(101);
plot(log.time,log.a1);
title('Position');
xlabel('time (sec)');
ylabel('position (rad)');
grid on;

% Plot the logged velocity feedback
figure(102);
plot(log.time,log.d1);
title('Velocity');
xlabel('time (sec)');
ylabel('velocity (rad/sec)');
grid on;


