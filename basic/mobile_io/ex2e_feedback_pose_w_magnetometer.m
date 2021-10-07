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
mobileIO = HebiMobileIO.findDevice('HEBI', 'mobileIO');
mobileIO.setDefaults();
mobileIO.setButtonIndicator(8);
mobileIO.sendText('Move for 30 seconds or press b8 to finish early');

%% Gather data w/ visualization
disp('  Visualizing 6-DoF pose estimate from the mobile device.');
disp('  Move the device over magnets to make the feedback interesting...');  

% Start logging in the background
mobileIO.group.startLog( 'dir', 'logs' );  

% Orientation visualization
frameDisplay = FrameDisplay();

isRunning = true;
t0 = tic();
while toc(t0) < 30 && isRunning
    
    % Visualize full 6-DoF pose 
    mobileIO.update();
    [pose, arQuality] = mobileIO.getArPose();
    frameDisplay.setFrames( pose );
    title(['AR Quality: ' num2str(arQuality)]);
    drawnow;
    
    isRunning = ~mobileIO.getFeedbackIO().b8;

end

disp('  All done!');

% Stop background logging
log = mobileIO.group.stopLogMobile();  

%% Analyze logged data
% Calculate magnetic field strength
fieldStrength = log.magnetometerX.^2 + ...
                log.magnetometerY.^2 +  ...
                log.magnetometerZ.^2; % [T]
fieldStrength = sqrt( fieldStrength ) * 1E6; % [uT]

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
title('Magnetic Field Strength');
axis equal;
colorbar;
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
view(2);
