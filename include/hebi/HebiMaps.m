classdef (Sealed) HebiMaps < handle
    
    % Public API
    methods(Access = public)
        
        function out = setPositionSource(this, varargin)
            out = setPositionSource(this.obj, varargin{:});
        end
        
        function out = setEncoder(this, varargin)
            out = setEncoder(this.obj, varargin{:});
        end
        
        function out = setUtSource(this, varargin)
            out = setUtSource(this.obj, varargin{:});
        end
        
        function out = setUtGates(this, varargin)
            out = setUtGates(this.obj, varargin{:});
        end
        
        function out = setWorldFrame(this, varargin)
            out = setWorldFrame(this.obj, varargin{:});
        end
        
        function out = setSurfaceMap(this, varargin)
            out = setSurfaceMap(this.obj, varargin{:});
        end
                
        function out = sendFeedbackRequest(this, varargin)
            out = sendFeedbackRequest(this.obj, varargin{:});
        end
        
        function varargout = getNextFeedback(this, varargin)
            varargout = cell(getNextFeedback(this.obj, varargin{:}));
        end
        
        function out = updateFeedback(this, varargin)
            out = updateFeedback(this.obj, varargin{:});
        end

        function out = startLog(this, varargin)
            out = startLog(this.obj, varargin{:});
        end

        function out = stopLog(this, varargin)
            out = stopLog(this.obj, varargin{:});
            
            % stop log should only be called in non-time critical sections,
            % so it may make sense to do some automated cleanup.
            if HebiMaps.config.triggerStopLogGC
                java.lang.System.gc();
            end
        end
        
    end
    
    methods(Access = public, Static)
        
         function out = convertLog(varargin)
            out = javaMethod('convertLog', HebiMaps.className, varargin{:});
            
            % stop log should only be called in non-time critical sections,
            % so it may make sense to do some automated cleanup.
            if HebiMaps.config.triggerStopLogGC
                java.lang.System.gc();
            end
        end
        
    end
    
    methods(Access = public, Hidden = true)
        
        function this = HebiMaps(varargin)
            % constructor
            this.obj = javaObject(HebiMaps.className);
            setAddress(this.obj, varargin{:});
        end
        
        function [] = delete(this)
            %destructor disposes this instance
            delete(this.obj);
        end
        
        function disp(this)
            %custom display
            disp(this.obj);
        end
        
        % Inherited methods that we don't want to see in
        % auto-complete or docs
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
    
    properties(Access = private, Hidden = true)
        obj
    end
    
    properties(Constant, Access = private, Hidden = true)
        className = hebi_load('HebiMaps');
        config = hebi_config('HebiGroup');
    end
    
    % Non-API Static methods for MATLAB compliance
    methods(Access = public, Static, Hidden = true)
        
        function varargout = methods(varargin)
            instance = javaObject(HebiGroup.className);
            switch nargout
                case 0
                    methods(instance, varargin{:});
                    varargout = {};
                otherwise
                    varargout{:} = methods(instance, varargin{:});
            end
        end
        
        function varargout = fields(varargin)
            instance = javaObject(HebiGroup.className);
            switch nargout
                case 0
                    fields(instance, varargin{:});
                    varargout = {};
                otherwise
                    varargout{:} = fields(instance, varargin{:});
            end
        end
        
    end
    
end
