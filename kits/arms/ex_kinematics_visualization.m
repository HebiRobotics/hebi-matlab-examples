%% Setup
clear *;
close all;

HebiLookup.initialize();

armName = '6-DoF + gripper';
armFamily = 'Arm';

[ armGroup, armKin, armParams ] = setupArm( armName, armFamily );

% Select whether coordinate frames for static links should be drawn as well
showLinkBodies = true;

% Length of the drawn axes
axisLength = 0.1; % [m]

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