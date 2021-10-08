% Setup robot kinematics, calculate inverse kinematics (IK), and visualize.
%
% For more information type:
%    help HebiKinematics
%
% HEBI Robotics
% July 2018

%%
clear *;
close all;

% Load the kinematics from HRDF file
kin = HebiUtils.loadHrdf('./hrdf/3-DoF_arm_example.hrdf');

% Initialize a helper utility to view the various coordinate frames
frameDisp = FrameDisplay();

% Inverse kinematics is where you specify a desired target pose for the 
% end-effector and calculate the joint angles that give you that pose.  
startXYZ = [ 0.5; -0.2; 0.1 ];  % [m]
endXYZ = [ 0.5; 0.2; 0.1 ];   % [m]

% Because the API uses local optimization to calculate IK, you need to give 
% an initial guess for the joint angles.  The guess should be something
% that looks similar to what you want the arm to look like, capturing
% things like "elbow-up" vs "elbow-down" configurations.  Generally, do not
% provide initial conditions that are near kinematic singularities, or 
% where all or most of the joint angles are 0.
initPositions = [-pi/4 pi/4 pi/2];  % "elbow up" solutions

% Calculate forward kinematics gradualy from the start position to the end
% postion and display.  THIS INTERPOLATION WILL BE A STRAIGHT LINE IN
% WORKSPACE.
stepSize = 0.01;

for step = 0:stepSize:1
    
    % Interpolate between start and end XYZ positions
    targetXYZ = (1-step)*startXYZ + step*endXYZ;

    positions = kin.getInverseKinematics( 'xyz', targetXYZ, ...
                                          'initial', initPositions );
    
    % If you're solving IK over and over in a loop, it is usually a good 
    % idea to seed with the last solution to improve speed and stability.
    initPositions = positions;                    
    
    % Calculate the FK so that we can visualize.
    armFrames = kin.getForwardKinematics( 'output', positions );
    
    frameDisp.setFrames( armFrames );
    title( ['End-Effector XYZ (m): ', num2str(targetXYZ')] );
    drawnow;
end















