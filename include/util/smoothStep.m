function [ xSmooth ] = smoothStep( x )
%SMOOTHSTEP takes a [0-1] in put and smooths it into and s-curve output.
% 
%  Implentation is acutally 'smootherstep', zeroing 1st and 2nd derivates at
%  the the end points.
%  https://en.wikipedia.org/wiki/Smoothstep
%
%  If you pass in anything outside of [0-1] the input gets clamped to be
%  with [0-1].  If you pass in a vector, a vector of smoothed inputs is 
%  returned.
% 
% Dave Rollinson
% Mar 2019

    % Clamp the input
    x = min( x, 1.0 );
    x = max( x, 0.0 );

    xSmooth = 6*x.^5 - 15*x.^4 + 10*x.^3;

end

