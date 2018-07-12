function [] = include()
% include adds paths required to run demos

localDir = fileparts(mfilename('fullpath'));
addpath(fullfile(localDir));

% Main API
addpath(fullfile(localDir, 'hebi'));
hebi_load();

% Joystick / Keyboard input
addpath(fullfile(localDir, 'input'));
HebiJoystick.loadLibs();
HebiKeyboard.loadLibs();

% MATLAB File Exchange
addpath(fullfile(localDir, 'exchange', 'SpinCalc'));

% Utilities
addpath(fullfile(localDir, 'kinematics'));
addpath(fullfile(localDir, 'util'));

end