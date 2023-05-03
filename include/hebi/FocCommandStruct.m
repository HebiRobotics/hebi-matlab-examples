function struct = FocCommandStruct()
    % FocCommandStruct is used to send commands to groups of modules.
    %
    %   The struct created by this function is used to command FOC
    %   (Field Oriented Control) values on motor drivers.
    %
    %   A FocCommandStruct only has to be initialized once, and the fields can
    %   be overwritten and reused multiple times in a loop.  When setting
    %   in a loop it is important to limit the loop rate using either a
    %   pause or a feedback request.
    %
    %   See also CommandStruct, IoCommandStruct, HebiGroup, HebiGroup.send

    %   Copyright 2022-2023 HEBI Robotics, Inc.
    struct = javaObject(hebi_load('FocCommandStruct'));
end
