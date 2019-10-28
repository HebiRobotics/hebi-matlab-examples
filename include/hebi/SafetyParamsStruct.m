function struct = SafetyParamsStruct()
% SafetyParamsStruct can be used to set safety parameters of groups
%
%   The struct created by this function can be used to set a variety of
%   safety limits and strategy parameters on a group of modules.
%
%   The online documentation provides more information about the 
%   different control parameters:
%   http://docs.hebi.us/core_concepts.html#safety_controller
%
%   SafetyParamStructs can be loaded and saved to an XML file format with 
%   functions provided in HebiUtils.  This format is the preferred way
%   of storing and loading parameters for different demos / applications.
%   More information on the XML format can be found at:
%   https://github.com/HebiRobotics/hebi-xml/blob/master/SAFETY.md
%
%   Empty entries in the struct and NaNs for any individual values in
%   safety parameters are ignored, i.e., any existing setting on the
%   device for that parameter will remain unmodified.
%
%   Example (Loading safety parameters from XML file and setting on a group):
%       safetyParams = HebiUtils.loadSafetyParams('mySafetyParams.xml');
%       group.send('SafetyParams', safetyParams);
%
%   Example (Saving safety parameters to XML file)
%       safetyParams = group.getSafetyParams();
%       HebiUtils.saveSafetyParams(safetyParams, 'mySafetyParams.xml');
%
%   Example (Manually setting safety limits):
%       limits = SafetyParamsStruct()
%       limits.positionMinLimit = [-pi -pi];
%       limits.positionMaxLimit = [+pi +pi];
%       group.send('SafetyParams', limits);
%
%   The 'mStopStrategy' and 'positionLimitStrategy' fields represent
%   enumerated values. You can find the available values in 
%
%       HebiEnum
%       HebiEnum
%
%   See also HebiGroup, HebiEnum, HebiUtils.saveSafetyParams, HebiUtils.loadSafetyParams

%   Copyright 2014-2019 HEBI Robotics, Inc.
struct = javaObject(hebi_load('SafetyParamsStruct'));
end
