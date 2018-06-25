function [] = startup()
% startup.m sets up libraries and paths and should be run once on startup.
% 
% HEBI Robotics
% Jun 2018

localDir = fileparts(mfilename('fullpath'));
includeScript = fullfile(localDir, '..', 'include', 'include.m');
run(includeScript);

end
