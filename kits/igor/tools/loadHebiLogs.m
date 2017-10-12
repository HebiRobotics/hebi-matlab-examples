function [ hebiLogs ] = loadHebiLogs( folderName, moduleSN, numLatestLogs, logView )
%LOADBREAKINLOGS Summary of this function goes here
%   Detailed explanation goes here

% Parse a bunch of logs and plot
%
% Dave Rollinson
% Feb 2017

    logFilePath = ['\\HEBINAS\Logs\X_Logs\' folderName '\' ,moduleSN '\'];

    fileInfo = dir( [logFilePath '*.hebilog'] );

    logFiles = {fileInfo.name};
    numLogs = length(logFiles);
    
    latestLogNums = 1:numLogs;
    if nargin >= 3
        latestLogNums = latestLogNums(end-numLatestLogs+1:end);
    end

    for i=1:length(latestLogNums)
        disp([ 'Loading ' num2str(latestLogNums(i)) ' of ' ...
               num2str(numLogs) ': '  logFiles{latestLogNums(i)} ' - ' ...
               num2str(fileInfo(latestLogNums(i)).bytes/1E6,'%0.1f') ' MB']);

        if nargin==4 && strcmp(logView,'full')
            hebiLogs{i} =  struct( HebiUtils.convertGroupLog( ...
                         [logFilePath logFiles{latestLogNums(i)}], ...
                         'view', 'full' ) );
        else
            hebiLogs{i} =  struct( HebiUtils.convertGroupLog( ...
                         [logFilePath logFiles{latestLogNums(i)}] ) );
        end
    end

end

