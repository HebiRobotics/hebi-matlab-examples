function [] = startup()
% startup.m sets up libraries and paths and should be run once on startup.
% 
% HEBI Robotics
% Jun 2018

localDir = fileparts(mfilename('fullpath'));

% Run the include script from the top level of the examples
includeScript = fullfile(localDir, '..', 'include', 'include.m');
run(includeScript);

% Add this folder and all its subfolders
addpath(fullfile(localDir, genpath('./') ));

end
