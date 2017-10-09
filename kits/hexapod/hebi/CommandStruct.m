function struct = CommandStruct()
    % CommandStruct can be used to send commands to groups
    %
    %   The struct created by this function can be used to command positions,
    %   velocities, and efforts to a group of modules. Setting the empty
    %   matrix or NaN for a particular value disables the corresponding
    %   control.
    %
    %   Example
    %       cmd = CommandStruct();
    %       cmd.position = [];
    %       cmd.velocity = [];
    %       cmd.effort = [0 0 0];
    %       group.send(cmd);
    %
    %   See also HebiLookup, HebiGroup
    
    %   Copyright 2014-2016 HEBI Robotics, LLC.
    struct = javaObject(hebi_load('CommandStruct'));
end
