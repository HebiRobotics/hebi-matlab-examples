function struct = SafetyParamsStruct()
% SafetyParamsStruct can be used to set safety parameters of groups
%
%   The struct created by this function can be used to set a variety of
%   safety limits and strategy parameters on a group of modules.
%
%   Empty entries in the struct and NaNs for any individual values in
%   safety parameters are ignored, i.e., any existing setting on the
%   device for that parameter will remain unmodified.
%
%   Example (manually setting safety limits):
%       limits = SafetyParamsStruct()
%       limits.positionMinLimit = [-pi -pi];
%       limits.positionMaxLimit = [+pi +pi];
%       group.send('SafetyParams', limits);
%
%   The 'mStopStrategy' and 'positionLimitStrategy' fields represent
%   enumerated values with the meaning below:
%
%   'mStopStrategy'
%       nan = Unknown
%        0  = Disabled
%        1  = Motor Off
%        2  = Motor Hold
%
%   'positionLimitStrategy'
%       nan = Unknown
%        0  = Disabled
%        1  = Motor Off
%        2  = Hold Position
%        3  = Damped Spring
%
%   See also HebiGroup, HebiUtils.send

%   Copyright 2014-2019 HEBI Robotics, Inc.
struct = javaObject(hebi_load('SafetyParamsStruct'));
end