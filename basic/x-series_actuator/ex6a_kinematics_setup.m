% Setup robot kinematics based on HEBI Robot Definition Format (HRDF) file.
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
kin = HebiKinematics('./hrdf/3-DoF_arm_example.hrdf');

% Display the basic kinematics information
disp(kin);