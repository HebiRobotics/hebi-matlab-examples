function struct = LocalStateStruct()
    % LocalStateStruct is used to set local state for logging purposes.
    %
    %   See also HebiGroup.setLocalState, HebiGroup.getLocalState
    
    %   Copyright 2014-2023 HEBI Robotics, Inc.
    struct = javaObject(hebi_load('LocalStateStruct'));
end
