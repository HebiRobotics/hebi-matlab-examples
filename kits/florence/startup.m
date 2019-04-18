function [] = startup()
% startup.m sets up libraries and paths and should be run once on startup.
% 
% HEBI Robotics
% Jun 2018

localDir = fileparts(mfilename('fullpath'));

% Add this folder and all its subfolders
addpath(genpath(localDir));

% Load the main API
hebi_load();

% Joystick / Keyboard 
% Load Libraries before making any other objects so that these tools 
% will work.  Libraries can no longer be loaded after Java objects of 
% any type have been instantiated during a Matlab session.
HebiJoystick.loadLibs();
HebiKeyboard.loadLibs();

end
