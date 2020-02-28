classdef HebiGripper < handle
    %HebiGripper interface for 
    %   Detailed explanation goes here
    
    properties
       openEffort = 1;
       closeEffort = -5;
       state = 0; % 0 is open, 1 is closed
    end
    
    properties(SetAccess = private)
        group HebiGroup;
    end
    
    properties(Access = private)
       cmd = CommandStruct(); 
    end
    
    methods
        
        function this = HebiGripper(group)
            %HebiGripper Construct an instance of this class
            %   Detailed explanation goes here
            this.group = group;
        end
        
        function newState = close(this)
            newState = this.setState(1);
        end
        
        function newState = open(this)
            newState = this.setState(0);
        end
        
        function state = toggle(this)
            state = this.setState(~this.state);
        end
        
        function state = setState(this, state)
            %SETSTATE sets gripper to a value between [0-1] where 0 is
            % fully open and 1 is fully closed. 'nan' is ignored.
            if state < 0 || state > 1
               error('Gripper state must be [0-1]'); 
            end
            if ~isnan(state)
                this.cmd.effort = ...
                    state * this.closeEffort + ...
                    (1-state) * this.openEffort;
                this.state = state;
            end
        end
        
        function [] = send(this)
            this.group.send(this.cmd);
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

