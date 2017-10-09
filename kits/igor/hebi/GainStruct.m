function struct = GainStruct()
    % GainStruct can be used to set gains of groups
    %
    %   The struct created by this function can be used to set a variety of
    %   gains on a group of modules. Please consult the online
    %   documentation to find out more about the individual gain settings.
    %
    %   Non-finite values and NaNs are ignored.
    %
    %   Example
    %       gains = GainStruct();
    %       gains.controlStrategy = 4;
    %       group.send('gains', gains);
    %
    %   See also HebiLookup, HebiGroup
    
    %   Copyright 2014-2016 HEBI Robotics, LLC.
    struct = javaObject(hebi_load('GainStruct'));
end