function [] = startup()
% startup adds paths required to run demos

localDir = fileparts(mfilename('fullpath'));
addpath(fullfile(localDir));
addpath(fullfile(localDir, 'hebi'));
addpath(fullfile(localDir, 'tools'));
addpath(fullfile(localDir, 'tools/gains'));
addpath(fullfile(localDir, 'tools/kinematics'));
addpath(fullfile(localDir, 'tools/input'));
addpath(fullfile(localDir, 'tools/visualization'));

HebiJoystick.loadLibs();
HebiKeyboard.loadLibs();

end