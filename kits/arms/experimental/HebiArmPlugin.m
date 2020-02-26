classdef HebiArmPlugin < handle
    % HebiArmPlugin interface for custom HebiArm plugins
    %   
    %   Plugins have access to the new and previous state. Plugins may
    %   update the new state as needed. Plugin specific state may also
    %   be stored in the state struct, but users are responsible for 
    %   avoiding variable name collisions.
    %
    %   The always accessible fields are:
    %   * time [s]
    %   * fbk [struct]
    %   * cmdPos [rad]
    %   * cmdVel [rad/s]
    %   * cmdAccel [rad/s^2]
    %   * cmdEffort [Nm|N]
    
    methods(Abstract)
        newState = update(this, newState, arm)
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

