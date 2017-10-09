classdef (Sealed) HebiPoseFilter
    %HebiPoseFilter is a simple filter that uses accel and gyro measurements to estimate the pose
    %
    %   This api is highly experimental and may change at any time.
    %
    %   HebiPoseFilter Methods:
    %      setMaxAccelNormDev - max deviation from the norm
    %      setMaxAccelWeight  - trust in accelerometers [0-1]
    %      setYaw             - sets dead-reackoned yaw [rad] (rotation about z in world)
    %      update             - input: 3x accel [g], 3x gyro [rad/s], abs time [s]
    %      getPose            - out: 4x4 transform matrix
    %
    %
    %   Example
    %      % Continuously find pose of a module
    %      group = HebiLookup.newGroupFromSerials('SA023');
    %      poseFilter = HebiPoseFilter();
    %      poseFilter.setYaw(0.23); % (optional) set yaw to non-zero origin
    %      while true
    %         % Update filter
    %         fbk = group.getNextFeedback();
    %         accels = [fbk.accelX, fbk.accelY, fbk.accelZ];
    %         gyros = [fbk.gyroX, fbk.gyroY, fbk.gyroZ];
    %         poseFilter.update(accels, gyros, fbk.time);
    %
    %         % Show current pose
    %         pose = poseFilter.getPose();
    %         disp(pose);
    %      end
    
    %   Copyright 2014-2016 HEBI Robotics, LLC.
    
    % Public API
    methods(Access = public)
        
        function this = setMaxAccelNormDev(this, varargin)
            %setMaxAccelNormDev max deviation from the norm
            setMaxAccelNormDev(this.obj, varargin{:});
        end
        
        function this = setMaxAccelWeight(this, varargin)
            %setMaxAccelWeight trust in accelerometers [0-1]
            setMaxAccelWeight(this.obj, varargin{:});
        end
        
        function this = setYaw(this, varargin)
            %setYaw sets dead-reackoned yaw (rotation about z in world)
            setYaw(this.obj, varargin{:});
        end
        
        function this = update(this, varargin)
            %update input: 1x3 accel [g], 1x3 gyro [rad/s], abs time [s]
            update(this.obj, varargin{:});
        end
        
        function out = getPose(this, varargin)
            % getPose returns the current pose in a transform matrix
            out = getPose(this.obj, varargin{:});
        end
        
    end
    
    properties(Access = private, Hidden = true)
        obj
    end
    
    properties(Constant, Access = private, Hidden = true)
        className = hebi_load('HebiPoseFilter');
    end
    
    % Non-API Methods for MATLAB compliance
    methods(Access = public, Hidden = true)
        
        function this = HebiPoseFilter()
            this.obj = javaObject(HebiPoseFilter.className);
        end
        
        function disp(this)
            disp(this.obj);
        end
        
    end
    
    % Non-API Static methods for MATLAB compliance
    methods(Access = public, Static, Hidden = true)
        
        function varargout = methods(varargin)
            instance = javaObject(HebiPoseFilter.className);
            switch nargout
                case 0
                    methods(instance, varargin{:});
                    varargout = {};
                otherwise
                    varargout{:} = methods(instance, varargin{:});
            end
        end
        
        function varargout = fields(varargin)
            instance = javaObject(HebiPoseFilter.className);
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