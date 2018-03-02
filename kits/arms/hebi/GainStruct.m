function struct = GainStruct()
    % GainStruct can be used to set gains of groups
    %
    %   The struct created by this function can be used to set a variety of
    %   gains on a group of modules. Please consult the online
    %   documentation to find out more about the individual gain settings.
    %
    %   NaNs for any values in gains or control parmaters are ignored. In
    %   these cases any existing setting on the module for that parameter
    %   will remain unmodified.
    %
    %   Example:
    %       % Set the control strategy of a 2 module group to strategy 4
    %       gains = GainStruct()
    %       gains.controlStrategy = [4 4];
    %       group.send('gains', gains);
    %
    %   See also HebiLookup, HebiGroup
    
    %   Copyright 2014-2017 HEBI Robotics, Inc.
    struct = javaObject(hebi_load('GainStruct'));
end