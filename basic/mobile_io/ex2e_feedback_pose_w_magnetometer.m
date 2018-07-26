% Combine the full pose and magnetometer feedback from a mobile device,
% visualize online, log in the background, and visualize the combined
% logged data.
%
% This script assumes that you have run 'startup.m' in this folder.
%
% HEBI Robotics
% July 2018

%% Setup
clear *;
close all;
HebiLookup.initialize(); 
group = HebiLookup.newGroupFromNames( 'HEBI', 'Virtual IO' );

%% Gather data w/ visualization
disp('  Visualizing 6-DoF pose estimate from the mobile device.');
disp('  Move the device over magnets to make the feedback interesting...');  

% Start logging in the background
group.startLog( 'dir', 'logs' );  

% Orientation visualization
frameDisplay = FrameDisplay();

t0 = tic();
while toc(t0) < 30
    
    % Read a struct of mobile specific sensor data
    fbk = group.getNextFeedbackMobile();

    % Get the orientation feedback and convert to rotation matrix so 
    % that it can be visualized.
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

% Stop background logging
log = group.stopLogMobile();  

%% Analyze logged data
% Calculate magnetic field strength
magnetometer = [
    log.magnetometerX ...
    log.magnetometerY ...
    log.magnetometerZ]; % [T]
fieldStrength = vecnorm(magnetometer,2,2) * 1E6; % [uT]

% Select only data with a good pose estimate
selected = (log.arQuality == 0);

% Visualize as 3D scatter plot where the color is based on the magnetic
% field strength
X = log.arPositionX(selected);
Y = log.arPositionY(selected);
Z = log.arPositionZ(selected);
S = 600 * ones(size(X));
C = fieldStrength(selected);
scatter3(X,Y,Z,S,C, '.');
axis equal;
view(2);
