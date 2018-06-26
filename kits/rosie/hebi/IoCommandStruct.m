function struct = IoCommandStruct()
    % IoCommandStruct can be used to send IO commands to groups
    %
    %   The struct created by this function can be used to set pins on
    %   an IO board. NaN and Inf values are ignored. Setting values
    %   to pins that are not configured to be output pins will be ignored
    %   by the hardware.
    %
    %   Example
    %       io = IoCommandStruct();
    %       io.a1 = 1;
    %       io.b3 = 0.3;
    %       group.send('io', io);
    %
    %   See also HebiLookup, HebiGroup
    
    %   Copyright 2014-2018 HEBI Robotics, LLC.
    struct = javaObject(hebi_load('IoCommandStruct'));
end