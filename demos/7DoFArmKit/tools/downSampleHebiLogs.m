function [ downsampledLogs ] = downSampleHebiLogs( ...
                                            inputLogs, downSampleFactor )
%DOWNSAMPLELOG lowers the sample rate of a log by a specified factor.
%Sample reduction is based just on sample order, not timestamps.
%
% downsampledLogs = downSampleLog( inputLogs, downSampleFactor ) 
%
% INPUTLOGS is either a log file or cell array of log files. DOWNSAMPLELOGS
% will match the type of INPUTLOGS, with the change that any log files that
% are Hebi Java classes will be converted to Matlab structs.
%
% DOWNSAMPLEFACTOR is a number that means 'keep every Nth sample'.  Values 
% should be integers greater than 2. For example a log that was sampled at 
% 100 Hz with DOWNSAMPLEFACTOR = 2 will return a log sampled at 
% approximately 50Hz.  
%
% Example:
%   smallLog = downSampleLog( myLog, 10 ); % Make a low-resolution log
%                                          % that keeps every 10th sample.
%
% See also PLOTHEBILOGS, LOADHEBILOGS, CROPHEBILOGS.
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

        sampleCount = rem(1:length(log.time),round(downSampleFactor));

        keepSamples = sampleCount==0;

        for j=1:length(logFields)
            log.(logFields{j}) = log.(logFields{j})(keepSamples,:);
        end
        
        downsampledLogs{i} = log;
    end
    
    if length(downsampledLogs)==1
        downsampledLogs = downsampledLogs{1};
    end

end

