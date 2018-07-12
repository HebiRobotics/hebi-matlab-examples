% Setup robot kinematics, calculate forward kinematics (FK), and visualize.
%
% For more information type:
%    help HebiKinematics
%
% HEBI Robotics
% July 2018

clear *;
close all;

% Load the kinematics from HRDF file
kin = HebiKinematics('/hrdf/3-DoF_arm_example.hrdf');

% Initialize a helper utility to view the various coordinate frames
frameDisp = FrameDisplay();

% Forward kinematics is where you use a set of joint angles to calculate
% the poses of various parts along the arm, usually the tip (end-effector).
startPositions = [0 0 0];
endPositions = [-pi 0 0];

% This will return the 4x4 homogeneous transform that describes the pose
% the end-effector in the world frame (in this case, the base module).  For
% more general information on frames kinematics see:
% http://docs.hebi.us/core_concepts.html#kinematics
endEffectorFrame = kin.getForwardKinematicsEndEffector( ...
                                            startPositions );
                                   
% Pull out the XYZ position from the transfrom
endEffectorXYZ = endEffectorFrame(1:3,4);

pause(2.0);

% Calculate
stepSize = .01;

for step = 0:stepSize:1
    positions = (1-step)*startPositions + step*endPositions;
   
    % This call returns the frames for all the bodies in the arm, which we
    % can use to visualize poses in 3D.  This verion of FK returns a set of
    % 4x4xN homogeneous transforms.  The last transform is the same as what
    % gets returned from .getForwardKinematicsEndEffector()
    armFrames = kin.getForwardKinematics( 'output', positions );
    endEffectorXYZ = armFrames(1:3,4,end);
    
    frameDisp.setFrames( armFrames );
    title( ['End-Effector XYZ (m): ', num2str(endEffectorXYZ')] );
    drawnow;
end









