function out = createFromConfig(pluginMap)
% createFromConfig is used to create arm plugins from a robot config
%
%   Example:
%       % instantiate config plugins
%       config = HebiUtils.loadRobotConfig('robot.yaml');
%       arm.plugins = HebiArmPlugins.createFromConfig(config.plugins);
%
%   See also HebiArm

%   Copyright 2023-2023 HEBI Robotics, Inc.

out = cell(0);
pluginNames = fields(pluginMap);
for i = 1:numel(pluginNames)

    cfg = pluginMap.(pluginNames{i});
    switch cfg.type

        case 'GravityCompensationEffort'
            plugin = HebiArmPlugins.GravityCompensation();
            if isfield(cfg, 'imu_feedback_index')
                plugin.imuFeedbackIndex = cfg.imu_feedback_index + 1;
            end
            if isfield(cfg, 'imu_frame_index')
                plugin.imuFrameIndex = cfg.imu_frame_index + 1;
            end
            if isfield(cfg, 'imu_rotation_offset')
                plugin.imuRotationOffset = reshape(cfg.imu_rotation_offset,3,3)';
            end

        case 'DynamicsCompensationEffort'
            plugin = HebiArmPlugins.DynamicsCompensation();

        case 'EffortOffset'
            plugin = HebiArmPlugins.EffortOffset(cfg.offset(:)');

        case 'ImpedanceController'
            plugin = HebiArmPlugins.ImpedanceController();
            plugin.gainsInEndEffectorFrame = cfg.gains_in_end_effector_frame;
            plugin.Kp = cfg.kp(:);
            plugin.Kd = cfg.kd(:);
            if isfield(cfg, 'ki')
                plugin.Ki = cfg.ki(:);
            end
            if isfield(cfg, 'i_clamp')
                plugin.iClamp = abs(cfg.i_clamp(:));
            end

        case 'DoubledJoint'
            error('The DoubledJoint plugin is not yet supported')

        otherwise
            warning(['Ignorning unknown plugin type: ' cfg.type])

    end

    % shared optional fields
    if isfield(cfg, 'enabled')
        plugin.enabled = logical(cfg.enabled);
    end
    if isfield(cfg, 'ramp_time')
        plugin.rampTime = double(cfg.ramp_time);
    end

    % save plugin
    out{end+1} = plugin;

end

end