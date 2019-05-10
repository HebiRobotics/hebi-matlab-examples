function [ smoothData ] = medianfilt( inputData, windowSize )
%SMOOTHFILT Takes in a vector of data and smooths it with a median over a
%specified size window.  It does this with zero-lag, looking forward and
%backwards in the vector.

    dataSize = size(inputData);

    if min(dataSize) > 1 || length(dataSize) > 2
        disp('Input data needs to be a Nx1 or 1xN vector.');
        return;
    end

    smoothData = nan(dataSize);
    dataLength = length(inputData);
    
    % Get the number of steps to either side of the data over which to
    % average.  For example, a window size of 5 would include the data
    % point itself, +2 and -2.  If the value is even, then it includes one
    % more point forward than it does backward.
    fwdWindow = ceil((windowSize-1) / 2);
    bwdWindow = floor((windowSize-1) / 2);
    
    for i=1:dataLength
        
        fwdIndex = min( i+fwdWindow, dataLength );
        bwdIndex = max( i-bwdWindow, 1 );

        smoothData(i) = median( inputData(bwdIndex:fwdIndex) );
    end
    
end

