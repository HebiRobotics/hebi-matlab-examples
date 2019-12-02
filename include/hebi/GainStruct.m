function struct = GainStruct()
% GainStruct can be used to set gains of groups
%
%   The struct created by this function can be used to set a variety of
%   gains and control parameters on a group of modules. A list of all 
%   currently available gains is below:
%
%                          time [s]
%               controlStrategy [HebiEnum]
%                 mStopStrategy [HebiEnum]
%                    positionKp
%                    positionKi
%                    positionKd
%                    positionFF
%              positionDeadZone
%                positionIClamp
%                 positionPunch
%             positionMinTarget
%             positionMaxTarget
%             positionMinOutput
%             positionMaxOutput
%     positionTargetLowpassGain
%     positionOutputLowpassGain
%              positionDOnError
%                    velocityKp
%                    velocityKi
%                    velocityKd
%                    velocityFF
%              velocityDeadZone
%                velocityIClamp
%                 velocityPunch
%             velocityMinTarget
%             velocityMaxTarget
%             velocityMinOutput
%             velocityMaxOutput
%     velocityTargetLowpassGain
%     velocityOutputLowpassGain
%              velocityDOnError
%                      effortKp
%                      effortKi
%                      effortKd
%                      effortFF
%                effortDeadZone
%                  effortIClamp
%                   effortPunch
%               effortMinTarget
%               effortMaxTarget
%               effortMinOutput
%               effortMaxOutput
%       effortTargetLowpassGain
%       effortOutputLowpassGain
%                effortDOnError
%
%   The online documentation provides more information about the 
%   individual gain settings and control parameters:
%   http://docs.hebi.us/core_concepts.html#controller_gains
%
%   GainStructs can be loaded and saved to an XML file format with 
%   functions provided in HebiUtils.  This format is the preferred way
%   of storing and loading gains for different demos / applications.  More
%   information on the XML format can be found at:
%   https://github.com/HebiRobotics/hebi-xml/blob/master/GAINS.md
%
%   Empty entries in the struct and NaNs for any individual values in 
%   gains or control parmaters are ignored. In these cases any existing 
%   setting on the module for that parameter will remain unmodified.  
%
%   Example (Loading gains from XML file and setting on a group)
%       gains = HebiUtils.loadGains('myGains.xml');
%       group.send('gains', gains);
%
%   Example (Saving gains to XML file)
%       gains = group.getGains();
%       HebiUtils.saveGains(gains, 'myGains.xml');
%
%   Example (Manually setting gains):
%       gains = GainStruct()
%       gains.controlStrategy = [4 4];
%       group.send('gains', gains);
%
%   See also HebiGroup, HebiEnum, HebiUtils.loadGains, HebiUtils.saveGains

%   Copyright 2014-2019 HEBI Robotics, Inc.
struct = javaObject(hebi_load('GainStruct'));
end