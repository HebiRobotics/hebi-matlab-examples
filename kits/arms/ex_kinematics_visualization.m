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

armName = 'A-2085-06_receive';
armFamily = 'Receive';
hasGasSpring = true;

[ arm, params ] = setupArm( armName, armFamily, hasGasSpring );

% Select whether coordinate frames for static links should be drawn as well
showLinkBodies = false;

localDir = params.localDir;
enableLogging = true;

% Length of the drawn axes
axisLength = 0.05; % [m]

%% Passive Visualization
selected = arm.kin.getBodyInfo().isDoF; 
if showLinkBodies
   selected(:) = true; 
end

frameDisplay = FrameDisplay(axisLength, sum(selected));
frameDisplay.drawLinks = true;

% Keyboard input
kb = HebiKeyboard();
keys = read(kb);

disp('Displaying coordinate frames for the bodies in the arm.');
disp('Press ESC to stop.');

% Start background logging 
if enableLogging
   logFile = arm.group.startLog( 'dir', [localDir '/logs'] ); 
end

while ~keys.ESC
    
    % Calculate kinematics based on latest feedback
    arm.update();
    frames = arm.kin.getForwardKinematics( 'OutputFrame', arm.state.fbk.position );
    
    % Draw coordinate frames
    frames = frames(:,:,selected);
    frameDisplay.setFrames(frames);
    drawnow;
    
    % Check for new key presses on the keyboard
    keys = read(kb);
    
end


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Stop background logging and visualize %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if enableLogging
    
   disp('Plotting logged feedback.');
   hebilog = arm.group.stopLogFull();
   
   % Plot the end-effectory trajectory and error
   kinematics_analysis( hebilog, arm.kin );
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   % Feel free to put more plotting code here %
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end