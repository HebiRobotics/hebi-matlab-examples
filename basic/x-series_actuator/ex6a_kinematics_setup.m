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
hrdfFileFolder = 'hrdf/';
hrdfFileName = '3-DoF_arm_example.hrdf';     % the '.hrdf' is optional
kin = HebiKinematics([hrdfFileFolder hrdfFileName]);

% Display the basic kinematics information
disp(kin);





