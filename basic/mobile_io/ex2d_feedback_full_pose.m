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
familyName = 'Android';
moduleNames = 'Phone';
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

group.startLog('dir','logs');

%% Visualize Full Pose
disp('  Visualizing 6-DoF pose estimate from the mobile device.');
disp('  Move it around to make the feedback interesting...');  

% Setup helper function to visualize the pose
frameDisplay = FrameDisplay();

duration = 30; % [sec]
timer = tic();
while toc(timer) < duration
    
    % Read a struct of mobile specific sensor data
    fbk = group.getNextFeedbackMobile();

    % Get the AR orientation feedback and convert to rotation matrix  
    % so that it can be visualized.
    mobileQuaterion = [ fbk.arOrientationW ...
                        fbk.arOrientationX ...
                        fbk.arOrientationY ...
                        fbk.arOrientationZ ];
    mobileRotMat = HebiUtils.quat2rotMat( mobileQuaterion );

    % Get the AR position feedback
    mobileXYZ = [ fbk.arPositionX;
                  fbk.arPositionY;
                  fbk.arPositionZ ];
   
    % Visualize full 6-DoF pose 
    arTransform = eye(4);
    arTransform(1:3,1:3) = mobileRotMat;
    arTransform(1:3,4) = mobileXYZ;
    frameDisplay.setFrames( arTransform );
    title(['AR Quality: ' num2str(fbk.arQuality)]);
    drawnow;

end

disp('  All done!');

log = group.stopLog('view','mobile');
