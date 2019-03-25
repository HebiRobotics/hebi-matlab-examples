function struct = GainStruct()
    % GainStruct can be used to set gains of groups
    %
    %   The struct created by this function can be used to set a variety of
    %   gains and control parameters on a group of modules.
    %
    %   GainStructs can be loaded and saved to an xml file format with 
    %   functions provided in HebiUtils.
    %
    %   Empty entries in the struct and NaNs for any individual values in 
    %   gains or control parmaters are ignored. In these cases any existing 
    %   setting on the module for that parameter will remain unmodified.  
    %
    %   The online documentation provides more information about the 
    %   individual gain settings and control parameters:
    %   http://docs.hebi.us/core_concepts.html#controller_gains
    %
    %   Example (loading gains):
    %       gains = HebiUtils.loadGains('myGains.xml');
    %       group.send('gains');
    %
    %   Example (saving gains)
    %       gains = group.getGains();
    %       HebiUtils.saveGains(gains, 'myGains.xml');
    %
    %   Example (manually setting gains):
    %       gains = GainStruct()
    %       gains.controlStrategy = [4 4];
    %       group.send('gains', gains);
    %
    %   See also HebiGroup, HebiUtils.loadGains, HebiUtils.saveGains
    
    %   Copyright 2014-2018 HEBI Robotics, Inc.
    struct = javaObject(hebi_load('GainStruct'));
end