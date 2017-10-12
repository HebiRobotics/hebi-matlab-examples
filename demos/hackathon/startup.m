function [] = startup()
% startup sets up libraries and should be run once on startup.

% find absolute path to library directory
currentDir = fileparts(mfilename('fullpath'));
addpath(fullfile(currentDir , 'hebi'));

% add utility directory
addpath(fullfile(currentDir, 'utils'));
addpath(fullfile(currentDir, 'tools'));

% explicitely pre-load libraries
hebi_load(); % main HEBI library
HebiKeyboard.loadLibs(); % Keyboard input

end