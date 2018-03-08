 function [ croppedLogs ] = cropHebiLogs( inputLogs, startTime, endTime )
%CROPHEBILOGS trims a log based on a starting and end time in seconds.
%
% croppedLogs = cropHebiLogs( inputLogs, startTime, endTime)
%
% INPUTLOGS is either a log file or cell array of log files.  CROPPEDLOGS
% will match the type of INPUTLOGS, with the change that any log files that
% are Hebi Java classes will be converted to Matlab structs.
%
% STARTIME and ENDTIME are timestamps for the beginning and end of the log
% crop. If you hand in empty [] for either time it will trim to the end of the
% log.
%
% Example:
%    croppedLog = cropHebiLogs( inputLog, [], 10); % Log file of the first
%                                                  % 10 seconds of data.
%
% See also PLOTHEBILOGS, LOADHEBILOGS, DOWNSAMPLEHEBILOGS.
%
% Dave Rollinson
% Feb 2015

    if ~iscell(inputLogs)
        hebiLogs{1} = inputLogs;
    else
        hebiLogs = inputLogs;
    end
        
    for i=1:length(hebiLogs)
        
        log = hebiLogs{i};
        
        if ~isstruct(log)
            disp('Converting log(s) from Java class to Matlab struct...');
            log = struct(log);
        end

        logFields = fields(log);

        if ~isempty(startTime)
            startIndex = find(log.time>startTime,1,'first');
        else
            startIndex = 1;
        end

        if ~isempty(endTime)
            endIndex = find(log.time<endTime,1,'last');
        else
            endIndex = length(log.time);
        end

        for j=1:length(logFields)
            log.(logFields{j}) = log.(logFields{j})(startIndex:endIndex,:);
        end
    
        croppedLogs{i} = log;
    end
    
    if length(croppedLogs)==1
        croppedLogs = croppedLogs{1};
    end

end

