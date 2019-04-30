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
moduleNames = 'Mobile IO';
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

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
    
    % Read a struct of mobile specific sensor data
    fbk = group.getNextFeedbackMobile();

    % Get the orientation feedback and convert to rotation
    % matrix so that it can be visualized.
    mobileQuaternion = [ fbk.orientationW ...
                         fbk.orientationX ...
                         fbk.orientationY ...
                         fbk.orientationZ ];
    mobileRotMat = HebiUtils.quat2rotMat( mobileQuaternion );

    % Visualize result
    frame = eye(4);
    frame(1:3,1:3) = mobileRotMat;
    frameDisplay.setFrames(frame) 
    drawnow;
   
end

disp('  All done!');