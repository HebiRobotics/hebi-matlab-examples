function [] = startup()
% startup adds paths required to run demos

localDir = fileparts(mfilename('fullpath'));
includeScript = fullfile(localDir, '..', '..', 'include', 'include.m');
run(includeScript);

end
