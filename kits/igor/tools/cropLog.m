 function [ log ] = cropLog( log, startTime, endTime )
%CROPLOG trims a log based on a starting and end time in seconds.
%
% log = cropLog( log, startTime, endTime)
%
% If you hand in empty [] for either time it will trim to the end of the
% log.  For example, CROPLOG( log, [], 10 ) will return a log of all the
% data from the first 10 seconds of the log.
%
% Dave Rollinson
% Feb 2015

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
    
    for i=1:length(logFields)
        log.(logFields{i}) = log.(logFields{i})(startIndex:endIndex,:);
    end

end

