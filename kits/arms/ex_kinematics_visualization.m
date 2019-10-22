% Kinematics Visualization Demo
%
% Features:      Connects to an arm and displays all the coordinate frames
%                all the arm's rigid bodies online based on the feedback
%                joint angles.
%
% Requirements:  MATLAB 2013b or higher
%
% Author:        Dave Rollinson
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
% Date:          Oct 2018

% Copyright 2017-2018 HEBI Robotics

%% Setup
clear *;
close all;

HebiLookup.initialize();

<<<<<<< Updated upstream
armName = '6-DoF R-Arm';
armFamily = 'R-Arm';
=======
armName = 'Luxembourg-Config1';
armFamily = 'Willig-Arm';
>>>>>>> Stashed changes

[ armGroup, armKin, armParams ] = setupArm( armName, armFamily );

% Select whether coordinate frames for static links should be drawn as well
showLinkBodies = false;

% Length of the drawn axes
axisLength = 0.05; % [m]

%% Passive Visualization
selected = armKin.getBodyInfo().isDoF; 
if showLinkBodies
   selected(:) = true; 
end

frameDisplay = FrameDisplay(axisLength, sum(selected));

% Keyboard input
kb = HebiKeyboard();
keys = read(kb);

disp('Displaying coordinate frames for the bodies in the arm.');
disp('Press ESC to stop.');

while ~keys.ESC
    
    % Calculate kinematics based on latest feedback
    fbk = armGroup.getNextFeedback();
    frames = armKin.getForwardKinematics( 'OutputFrame', fbk.position );
    
    % Draw coordinate frames
    frames = frames(:,:,selected);
    frameDisplay.setFrames(frames);
    drawnow;
    
    % Check for new key presses on the keyboard
    keys = read(kb);
    
end