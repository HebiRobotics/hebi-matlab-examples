function struct = CommandStruct()
    % CommandStruct is used to send commands to groups of modules.
    %
    %   The struct created by this function is used to command positions,
    %   velocities, and efforts to a group of modules. You can command any
    %   combination of positions, velocities, and efforts simultaneously.
    %   The details on how simultaneous commands get combined on the module
    %   will depend on the gains and control parameters on the module.
    %   These gains can be set using a GainStruct and HebiGroup.send().
    %
    %   Setting the empty matrix [] disables control for all modules for 
    %   that command.  When you initialize a new CommandStruct the fields 
    %   for position, velocity, and effort are all empty by default.
    % 
    %   Setting any individual command to NaN will disable the controller
    %   on that specific module for that command.
    %
    %   Note that commands sent to a module will only be active for as long
    %   as the commandLifetime, or until a new command is sent, whichever
    %   comes first.
    %
    %   A CommandStruct only has to be initialized once, and the fields can
    %   be overwritten and reused multiple times in a loop.  When setting
    %   in a loop it is important to limit the loop rate using either a
    %   pause or a feedback request, see examples below.
    %
    %   Examples:
    %       % Send zero-effort commmands to a group of 3 modules
    %       cmd = CommandStruct();
    %       tic;
    %       while toc < 5
    %           cmd.effort = [0 0 0];
    %           group.send(cmd);
    %           pause(.01);     % Use pause to limit loop rate
    %       end
    %
    %       % Send positiona and velocity commands to all modules in a 
    %       % group, automatically finding the proper number of modules.
    %       numModules = group.getNumModules;
    %       cmd = CommandStruct();
    %       cmd.position = zeros(1,numModules);
    %       cmd.velocity = zeros(1,numModules);
    %       cmd.effort = [];
    %       tic;
    %       while toc < 10
    %           fbk = group.getNextFeedback();  % Use getNextFeedback() to
    %                                           % limit loop rate.
    %           group.send(cmd);
    %       end
    %
    %       % Send velocity commands to the first and last modules in a
    %       % 3-module group, but not the second.
    %       cmd = CommandStruct();
    %       tic;
    %       while toc < 15
    %           cmd.velocity = [1 nan 1];
    %           pause(.01);     % Use pause to limit loop rate
    %           group.send(cmd);
    %       end
    %
    %   See also HebiLookup, HebiGroup, GainStruct, HebiGroup.send,
    %   HebiGroup.setCommandLifetime.
    
    %   Copyright 2014-2018 HEBI Robotics, Inc.
    struct = javaObject(hebi_load('CommandStruct'));
end
