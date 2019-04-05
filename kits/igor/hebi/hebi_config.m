function [config] = hebi_config(simpleName)
%HEBI_CONFIG exposes default parameters that can be edited if needed

%   Copyright 2014-2018 HEBI Robotics, Inc.
switch simpleName
    
    case 'hebi_load'
        % Path to the binary library file. By default, it is assumed that
        % the library is placed in the same folder as this file.
        config.libraryVersion = 'hebi-matlab-1.5.1';
        config.libraryPath = fileparts(mfilename('fullpath'));
        
    case 'HebiLookup'
        % The default '*' broadcasts on all interfaces. If your network
        % does not support broadcast, or you only want to send lookup
        % requests to a limited number of devices, you can specify ip
        % addresses manually, e.g.,
        %     addresses = {'10.10.10.1', '10.10.10.2', '10.10.10.3'};
        config.defaultLookupAddresses = '*';
        
        % Sets the initial request rate of the lookup. This affects the
        % rate at which meta info get updated, e.g., names and ip
        % addresses. In the default mode, this also affects how fast
        % gains get updated in groups.
        config.defaultLookupFrequency = 5; % [Hz]
        
        % The initial frequency at which group.getNextFeedback() can run.
        config.defaultInitialGroupFeedbackFrequency = 100; % [Hz]
        
        % The command lifetime is the duration for which a command remains
        % active. If supporting hardware does not receive further commands
        % within the specified time frame, all local controllers get
        % deactivated. This is a safety feature to mitigate the risk of 
        % accidents on unexpected program interruptions. Set to inf or the
        % empty matrix [] to deactivate.
        config.defaultInitialGroupCommandLifetime = 0.25; % [s]
        
        % An initial pause to give the lookup time to find modules on the 
        % network. This pause is only executed at the first call and does
        % not affect subsequent calls. Slow or unreliable networks may need
        % a longer pause.
        config.initialNetworkLookupPause = 0.5; % [s]
        
    case 'HebiGroup'
        % Triggers a garbage collection after calls to stop logging. This
        % is usually done in non-time critical sections, so it is often a
        % convenient spot to do some cleanup.
        config.triggerStopLogGC = true;
        
    case 'HebiTrajectoryGenerator'
        % Minimum time to move between the start and end points of a
        % trajectory. If this value is too low, the accelerations may be
        % faster than physically possible.
        config.defaultMinDuration = 2.0; % [s]
        
        % Speed factor that gets applied to all trajectories. For
        % example, a value of 0.5 would slow down all trajectories to 50%.
        config.defaultSpeedFactor = 1;
        
        % Algorithm used to calculate trajectories
        config.defaultAlgorithm = 'UnconstrainedQp';
        
    otherwise
        config = [];
        
end
end