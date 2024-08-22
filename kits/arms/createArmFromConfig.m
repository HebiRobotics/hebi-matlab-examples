function arm = createArmFromConfig( config )

    %% Create models according to config
    % Comms
    group = HebiLookup.newGroupFromNames(config.families, config.names);

    % Kinematic Model
    kin = HebiUtils.loadHRDF(config.hrdf);

    % Load gain files
    gains = HebiUtils.loadGains(config.gains.default);

    %% Common Setup
    arm = HebiArm(group, kin);
    arm.plugins = HebiArmPlugin.createFromConfigMap(config.plugins);
    HebiUtils.sendWithRetry(arm.group, 'gains', gains);
end