function mobileIO = createMobileIOFromConfig( config )

    % Required keys for the mobile IO configuration
    requiredKeys = {'mobile_io_family', 'mobile_io_name', 'mobile_io_layout'};

    % Validate the mobile_io configuration
    if all(isfield(config.userData, requiredKeys))
        
        % Check that all required fields are present and are non-empty strings
        if all(cellfun(@(key) ischar(config.userData.(key)), requiredKeys))
            
            mobileIO = HebiMobileIO.findDevice(config.userData.mobile_io_family, config.userData.mobile_io_name);
            config.userData.mobile_io_layout
            mobileIO.group.send('MobileLayout', fullfile('config', config.userData.mobile_io_layout));

        else
            error('HEBI config "user_data"''s fields %s must contain strings, not parseable as strings', strjoin(requiredKeys, ', '));
        end
    else
        error('HEBI config "user_data" field must contain the keys: %s', strjoin(requiredKeys, ', '));
    end
end