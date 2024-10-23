classdef HebiArmPlugin < handle
    % HebiArmPlugin interface for custom HebiArm plugins
    %   
    %   Plugins have access to the arm state and may modify it as needed.
    %   Plugin specific state may also be stored in the state struct, 
    %   but users are responsible for avoiding variable name collisions.
    %
    %   The always accessible fields are:
    %   * time [s]
    %   * dt [s] (time since last call, or 0 on first run)
    %   * fbk [struct]
    %   * cmdPos [rad]
    %   * cmdVel [rad/s]
    %   * cmdAccel [rad/s^2]
    %   * cmdEffort [Nm|N]
    %
    % See also HebiArm
    
    %   Copyright 2023-2023 HEBI Robotics, Inc.

    properties(SetAccess = protected)
        % type of the plugin
        type char = '';
    end

    properties(Access = public)

        % whether the plugin should be run
        enabled logical = true; 

        % time in seconds to apply full effects after enabling. zero
        % indicates no ramping
        rampTime double = 0;

        % name of the plugin
        name char = '';

    end

    properties(Access = protected)

        % current enabled ratio
        enabledRatio double = 0;

        % target ratio
        targetRatio double = 1;

        % Timestamp when the ramping started
        lastTimestamp = [];

    end
    
    methods(Abstract)
        [] = update(this, arm)
    end

    methods (Access = protected)

        function scale = getRampScale(this, timestamp)

            % Figure out the step delta (dt / rampTime)
            if (isempty(this.lastTimestamp))
                delta = 0;
            else
                dt = timestamp - this.lastTimestamp;
                delta = dt / this.rampTime;
            end
            this.lastTimestamp = timestamp;

            % Advance enabled ratio
            if (this.enabledRatio < this.targetRatio)
                this.enabledRatio = min(this.enabledRatio + delta, 1);
            elseif (this.enabledRatio > this.targetRatio)
                this.enabledRatio = max(this.enabledRatio - delta, 0);
            end

            % Current scale
            scale = this.enabledRatio;

        end

    end

    methods
        
        % Setter for enabled property. Resets the ramp timer
        % whenever the value gets set.
        function [] = set.enabled(this, enabled)
            this.targetRatio = 1.0 * logical(enabled);
            this.enabled = enabled;
        end
        
    end
    
    methods (Access = public, Static)
        
        function out = createFromConfigMap(pluginMap)
            % createFromConfigMap loads plugins from a robot config plugin map
            %
            %   Example:
            %       % instantiate config plugins
            %       config = HebiUtils.loadRobotConfig('robot.yaml');
            %       arm.plugins = HebiArmPlugin.createFromConfigMap(config.plugins);
            %
            % See also HebiArm, HebiUtils.loadRobotConfig
            
            pluginNames = fields(pluginMap);
            out = struct();

            for i = 1:numel(pluginNames)
                plugin = HebiArmPlugin.createFromConfig(pluginMap.(pluginNames{i}));
                out.(plugin.name) = plugin; % Use plugin's name as the struct field name
            end
                
        end
        
    end
    
    methods (Access = private, Static)
        
        function plugin = createFromConfig(cfg)
            % createFromConfig loads a single plugin from a robot config
            %
            % See also HebiArm, HebiArmPlugin.createPluginsFromConfig
            switch cfg.type
                
                case 'GravityCompensationEffort'
                    plugin = HebiArmPlugins.GravityCompensation();
                    if isfield(cfg, 'imu_feedback_index')
                        plugin.imuFeedbackIndex = cfg.imu_feedback_index; % zero indexed
                    end
                    if isfield(cfg, 'imu_frame_index')
                        plugin.imuFrameIndex = cfg.imu_frame_index; % zero indexed
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
                    group = HebiLookup.newGroupFromNames(cfg.group_family, cfg.group_name);
                    plugin = HebiArmPlugins.DoubledJoint(cfg.index, group, cfg.mirror);
                    
                otherwise
                    warning(['Ignorning unknown plugin type: ' cfg.type])
                    
            end
            
            % name
            plugin.name = cfg.name;

            % type (TODO: set these directly in the subclasses)
            plugin.type = cfg.type;
            
            % shared optional fields
            if isfield(cfg, 'enabled')
                plugin.enabled = logical(cfg.enabled);
            end
            if isfield(cfg, 'ramp_time')
                plugin.rampTime = double(cfg.ramp_time);
            end
        end
        
    end
    
    % Inherited methods that we don't want to see in auto-complete
    % or generated documentation.
    methods(Access = public, Hidden = true)
        function varargout = addlistener(varargin)
            varargout{:} = addlistener@handle(varargin{:});
        end
        function varargout = eq(varargin)
            varargout{:} = eq@handle(varargin{:});
        end
        function varargout = findobj(varargin)
            varargout{:} = findobj@handle(varargin{:});
        end
        function varargout = findprop(varargin)
            varargout{:} = findprop@handle(varargin{:});
        end
        function varargout = ge(varargin)
            varargout{:} = ge@handle(varargin{:});
        end
        function varargout = gt(varargin)
            varargout{:} = gt@handle(varargin{:});
        end
        function varargout = le(varargin)
            varargout{:} = le@handle(varargin{:});
        end
        function varargout = listener(varargin)
            varargout{:} = listener@handle(varargin{:});
        end
        function varargout = lt(varargin)
            varargout{:} = lt@handle(varargin{:});
        end
        function varargout = ne(varargin)
            varargout{:} = ne@handle(varargin{:});
        end
        function varargout = notify(varargin)
            varargout{:} = notify@handle(varargin{:});
        end
    end
    
end

