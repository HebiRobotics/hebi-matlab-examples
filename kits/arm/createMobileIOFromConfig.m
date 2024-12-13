function mobileIO = createMobileIOFromConfig( config )
    userData = config.userData;

    % Required keys for the mobile IO configuration
    requiredKeys = {'mobile_io_family', 'mobile_io_name', 'mobile_io_layout'};

    % Validate the user data configuration
    if ~all(isfield(userData, requiredKeys))
        error('HEBI config "user_data" field must contain the keys: %s', strjoin(requiredKeys, ', '));
    end
    if ~all(cellfun(@(key) ischar(userData.(key)), requiredKeys))
        error('HEBI config "user_data" fields %s must contain strings', strjoin(requiredKeys, ', '));
    end

    % Load layout relative to the config file
    [cfgDir, ~, ~] = fileparts(config.configLocation);
    layoutFile = fullfile(cfgDir, userData.mobile_io_layout);

    % Validate the mobile_io configuration
    mobileIO = HebiMobileIO.findDevice(userData.mobile_io_family, userData.mobile_io_name);
    mobileIO.sendLayout(layoutFile);
    mobileIO.update();

end