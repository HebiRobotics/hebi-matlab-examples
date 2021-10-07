% Get orientation feedback from a mobile device and visualize online
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

mobileIO.group.startLog('dir','logs');

%% Visualize Device Orientation
disp('  Visualizing orientation estimate from the mobile device.');
disp('  Move it around to make the feedback interesting...');  

% Setup helper function to visualize the orientation
xyzLimits = [ -0.05 0.05;
              -0.05 0.05;
              -0.05 0.05 ];
frameDisplay = FrameDisplay( [], [], xyzLimits );

duration = 30; % [sec]
timer = tic();
while toc(timer) < duration
    
    % Get the orientation feedback 
    mobileIO.update();  
    mobileRotMat = mobileIO.getOrientation();

    % Visualize result
    frame = eye(4);
    frame(1:3,1:3) = mobileRotMat;
    frameDisplay.setFrames(frame) 
    drawnow;
   
end

disp('  All done!');

log = mobileIO.group.stopLogMobile();

