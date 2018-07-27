% Get full pose feedback from a mobile device, visualize online, log in 
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
familyName = 'Android';
moduleNames = 'Phone';
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

group.startLog('dir','logs');

%% Visualize Full Pose
disp('  Visualizing 6-DoF pose estimate from the mobile device.');
disp('  Move it around to make the feedback interesting...');  

% Setup helper function to visualize the orientation
mobilePose = FrameDisplay();
frames = eye(4);

figure(1);
clf;

duration = 60; % [sec]
timer = tic();
while toc(timer) < duration
    
    fbk = group.getNextFeedbackMobile();

    % Get the orientation feedback and convert to rotation matrix so that it
    % can be visualized.
    mobileQuaterion = [ fbk.arOrientationW ...
                        fbk.arOrientationX ...
                        fbk.arOrientationY ...
                        fbk.arOrientationZ ];
    mobileRotMat = HebiUtils.quat2rotMat( mobileQuaterion );

    mobileXYZ = [ fbk.arPositionX;
                  fbk.arPositionY;
                  fbk.arPositionZ ];
   
    arTransform = eye(4);
    arTransform(1:3,1:3) = mobileRotMat;
    arTransform(1:3,4) = mobileXYZ;
          
    mobilePose.setFrames( arTransform );
    title(['AR Quality: ' num2str(fbk.arQuality)]);
    drawnow;

end

disp('  All done!');

log = group.stopLog('view','mobile');
