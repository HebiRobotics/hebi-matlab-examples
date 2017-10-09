function [ log ] = downSampleLog( log, downSampleFactor )
%DOWNSAMPLELOG lowers the sample rate of a log by a specified factor
%
% log = downSampleLog( log, downSampleFactor )
%
% For example a log that was sampled at 100 Hz with downSampleFactor=2
% will return a log sampled at approximately 50Hz.  Values should be 
% integers greater than 2.
%
% Dave Rollinson
% Feb 2015

    if ~isstruct(log);
        disp('Converting log from Java class to Matlab struct...');
        log = struct(log);
    end

    logFields = fields(log);
    
    sampleCount = rem(1:length(log.time),round(downSampleFactor));
    
    keepSamples = sampleCount==0;

    for i=1:length(logFields)
        log.(logFields{i}) = log.(logFields{i})(keepSamples,:);
    end

end

