function gripper = createGripperFromConfig( config )
    userData = config.userData;

    % Required keys for the gripper configuration
    requiredKeys = {'gripper_family', 'gripper_name', 'gripper_gains', 'gripper_close_effort', 'gripper_open_effort'};
    requiredKeysString = {'gripper_family', 'gripper_name', 'gripper_gains'};
    requiredKeysFloat = {'gripper_close_effort', 'gripper_open_effort'};

    % Validate the user data configuration
    if ~all(isfield(userData, requiredKeys))
        error('HEBI config "user_data" field must contain the keys: %s', strjoin(requiredKeys, ', '));
    end
    if ~all(cellfun(@(key) ischar(userData.(key)), requiredKeysString))
        error('HEBI config "user_data" fields %s must contain strings', strjoin(requiredKeysString, ', '));
    end
    if ~all(cellfun(@(key) isnumeric(userData.(key)), requiredKeysFloat))
        error('HEBI config "user_data" fields %s must contain numbers', strjoin(requiredKeysFloat, ', '));
    end

    % Load gains relative to the config file
    [cfgDir, ~, ~] = fileparts(config.configLocation);
    gainsFile = fullfile(cfgDir, userData.gripper_gains);
    gains = HebiUtils.loadGains(gainsFile);

    % Create the backing group comms
    group = HebiLookup.newGroupFromNames(userData.gripper_family, userData.gripper_name);
    HebiUtils.sendWithRetry(group, 'gains', gains);

    % Setup gripper object
    gripper = HebiGripper(group);
    gripper.openEffort = userData.gripper_open_effort;
    gripper.closeEffort = userData.gripper_close_effort;

end