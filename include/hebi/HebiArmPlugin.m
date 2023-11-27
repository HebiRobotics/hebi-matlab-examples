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

    properties(Access = public)

        % whether the plugin should be run
        enabled logical = true; 

        % time in seconds to apply full effects after enabling. zero
        % indicates no ramping
        rampTime double = 0;

    end

    properties(Access = protected)

        % Timestamp when the ramping started
        rampStartTime = [];

    end
    
    methods(Abstract)
        [] = update(this, arm)
    end

    methods (Access = protected)

        function scale = getRampScale(this, timestamp)

            % Initialize ramp timestamp
            if isempty(this.rampStartTime)
                this.rampStartTime = timestamp;
            end

            % Determine the scale for the current time
            elapsedTime = timestamp - this.rampStartTime;
            if elapsedTime >= this.rampTime
                scale = 1; % fully ramped up
            else
                scale = elapsedTime / this.rampTime; % gradual ramping
            end

        end

    end

    methods 

        % Setter for enabled property. Resets the ramp timer
        % whenever the value gets set.
        function [] = set.enabled(this, enabled)
            this.rampStartTime = [];
            this.enabled = enabled;
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

