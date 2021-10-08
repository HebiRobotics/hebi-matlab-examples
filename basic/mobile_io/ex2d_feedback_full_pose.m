% Get full pose feedback from a mobile device and visualize online
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
mobileIO.initializeUI();

mobileIO.group.startLog('dir','logs');

%% Visualize Full Pose
disp('  Visualizing 6-DoF pose estimate from the mobile device.');
disp('  Move it around to make the feedback interesting...');  

% Setup helper function to visualize the pose
frameDisplay = FrameDisplay();

duration = 30; % [sec]
timer = tic();
while toc(timer) < duration
    
    % Update Feedback
    mobileIO.update();  
    [arTransform, arQuality] = mobileIO.getArPose();
    
    frameDisplay.setFrames( arTransform );
    title( ['AR Quality: ' num2str(arQuality)] );
    drawnow;

end

disp('  All done!');

log = mobileIO.group.stopLogMobile();
