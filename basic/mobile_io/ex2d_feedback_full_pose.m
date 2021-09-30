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
deviceName = 'Mobile IO';

% Loop to keep trying to form the arm group.  Make sure the mobile device
% has the correct name / family to match the ones above and that it is on
% the same network as this computer.  You can check for this using Scope.
while true  
    try
        fprintf('Searching for phone Controller...\n');
        group = HebiLookup.newGroupFromNames( ...
                        familyName, deviceName );        
        disp('Phone Found.  Starting up');
        break;
    catch
        % If we failed to make a group, pause a bit before trying again.
        pause(1.0);
    end
end

mobileIO = HebiMobileIO( group );
mobileIO.setDefaults();

group.startLog('dir','logs');

%% Visualize Full Pose
disp('  Visualizing 6-DoF pose estimate from the mobile device.');
disp('  Move it around to make the feedback interesting...');  

% Setup helper function to visualize the pose
frameDisplay = FrameDisplay();

duration = 30; % [sec]
timer = tic();
while toc(timer) < duration
    
    % Get Feedback
    mobileIO.update();  
    fbkMobile = mobileIO.getFeedbackMobile();
  
    [arTransform, arQuality] = mobileIO.getArPose();
    
    frameDisplay.setFrames( arTransform );
    title( ['AR Quality: ' num2str(arQuality)] );
    drawnow;

end

disp('  All done!');

log = group.stopLog('view','mobile');
