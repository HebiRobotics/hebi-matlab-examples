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

    plugin = pluginMap.(pluginNames{i});
    switch plugin.type

        case 'EffortOffset'
            out{end+1} = HebiArmPlugins.EffortOffset(plugin.effortOffset(:)');

        otherwise
            warning(['Ignorning unknown plugin type: ' plugin.type])

    end

end

end