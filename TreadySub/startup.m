function [] = startup()
% startup sets up libraries and should be run once on startup.

localDir = fileparts(mfilename('fullpath'));
includeScript = fullfile(localDir, '..', 'include', 'include.m');
run(includeScript);


end
