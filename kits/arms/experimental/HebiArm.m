classdef HebiArm < handle
    %HEBIARM Summary of this class goes here
    %   Detailed explanation goes here
    %
    %
    % % TODO:
    % * add arm.initialize() or arm.moveHome()?
    % * we need to ramp-up/soft-start. Scale gains?
    % * remove duplicate waypoints from auto-timing
    % * add a data type to store waypoints, e.g., a list-like structure
    %     wpts = HebiWaypoints();
    %     wpts.add(fbk.position, aux.state);
    %     List<struct<armState, auxState>> waypoints;
    % * speed factor? auto-timing?
    
    properties(SetAccess = private)
        group HebiGroup;
        kin HebiKinematics;
        trajGen HebiTrajectoryGenerator;
        traj HebiTrajectory;
        
        state struct = [];
    end
    
    properties(Access = public)
        plugins cell = {};
    end
    
    properties(Access = private)
        cmd = CommandStruct();
        trajStartTime double = 0;
        nZeros double;
    end
    
    methods
        
        function this = HebiArm(varargin)
            if length(varargin) ~= 2
               error('expected 2 arguments: group, kinematics'); 
            end
            this.group = varargin{1};
            this.kin = varargin{2};
            
            this.trajGen = HebiTrajectoryGenerator(this.kin);
            this.trajGen.setMinDuration(1.00); % min move time for 'small' movements (default 1.0)
            this.trajGen.setSpeedFactor(0.75); % slow down to safer speed (default = 1.0)
            
            this.nZeros = zeros(1, this.kin.getNumDoF);
        end
        
        function [] = setGains(this, gains)
            freq = this.group.getFeedbackFrequency();
            finally = onCleanup(@() this.group.setFeedbackFrequency(freq));
            this.group.setFeedbackFrequency(0);
            
            maxRetries = 10;
            while ~this.group.send('gains', gains, 'RequestAck', true)
                maxRetries = maxRetries - 1;
                if maxRetries == 0
                    error('Failed to verify setting gains due to timeouts');
                end
            end
        end
        
        function [] = cancelGoal(this)
            % CANCELGOAL cancels any active goal, returning to a
            % "weightless" state that does not actively command position
            % or velocity
            this.traj = [];
        end

        function [] = setGoal(this, varargin)
            %arm.setGoal('positions', pos, 'velocities', vel, 'accelerations', accel, 'time', time); 
            % Calculates trajectory from current pos (fbk) to target
            % same as newJointMove, but includes current state? [cmdPos pos] => traj generator
            %
            % Note that update() must be called before setGoal()!
            
            if isempty(this.state)
               error('update() must be called at least once before setGoal()'); 
            end
            
            % Recalculates the active trajectory from the last commanded
            % state to the set goal waypoints
            parser = inputParser();
            parser.addRequired('Positions');
            parser.addOptional('Velocities', []);
            parser.addOptional('Accelerations', []);
            parser.parse(varargin{:});
            goal = parser.Results;
            
            % Set default constraints if unspecified            
            defaultConstraints = nan * goal.Positions;
            defaultConstraints(end,:) = 0;
            if isempty(goal.Velocities)
                goal.Velocities = defaultConstraints;
            end
            if isempty(goal.Accelerations)
                goal.Accelerations = defaultConstraints;
            end
                    
            % Start the trajectory from the current state
            if isempty(this.traj) || isempty(this.state.cmdPos)
                fbk = this.group.get('feedback');
                cmdPos = fbk.position;
                cmdVel = 0 * cmdPos;
                cmdAccel = 0 * cmdPos;
                startTime = fbk.time;
            else
                cmdPos = this.state.cmdPos;
                cmdVel = this.state.cmdVel;
                cmdAccel = this.state.cmdAccel;
                startTime = this.state.time;
            end
            
            % Create trajectory starting from last known state 
            % (TODO: timing / duration?)
            % TODO: Time starts always at zero. Passing in zero time would throw
            % a warning unless start is exactly the same (to some epsilon)
            this.traj = this.trajGen.newJointMove(...
                [cmdPos; goal.Positions], ...
                'Velocities', [cmdVel; goal.Velocities], ...
                'Accelerations', [cmdAccel; goal.Accelerations]);
            this.trajStartTime = startTime;
            
        end
        
        function value = isAtGoal(this)
            value = this.getProgress() >= 1;
        end
        
        function [progress] = getProgress(this)
            % returns normalized [0-1] time of where trajectory is
            % currently at
            if isempty(this.traj) || isempty(this.state)
                error('trajectory not initialized');
            end
            
            t = this.state.time-this.trajStartTime;
            progress = min(t / this.traj.getDuration(),1);
            
        end
        
        function [] = update(this)
            % reads feedback and updates internal state
            
            % Read feedback
            newState = struct();
            newState.fbk = this.group.getNextFeedbackFull();
            newState.time = newState.fbk.time;
            newState.numDoF = this.kin.getNumDoF();
            
            % Always compensate for accelerations due to gravity
            newState.cmdEffort = this.getGravCompEfforts(newState.fbk);
            
            % Add pos/vel/accel commands
            if ~isempty(this.traj)
                
                % Evaluate trajectory state
                t = min(newState.time - this.trajStartTime, this.traj.getDuration());
                [ newState.cmdPos,  newState.cmdVel,  newState.cmdAccel] = this.traj.getState(t);
                
                % Compensate for joint accelerations
                newState.cmdEffort = this.kin.getDynamicCompEfforts(...
                    newState.fbk.position, ...
                    newState.cmdPos, ...
                    newState.cmdVel, ...
                    newState.cmdAccel) + newState.cmdEffort;
                
            else
                newState.cmdPos = [];
                newState.cmdVel = [];
                newState.cmdAccel = [];
            end
            
            % Call plugins (FK, Jacobians, End-Effector XYZ, etc.)
            for i=1:length(this.plugins)
                newState = this.plugins{i}.update(newState, this.state);
            end            
            this.state = newState;
            
            % TODO: --> user code
            % * add effort offsets for e.g. spring.
            % * add efforts to add an end effector wrench (local or world coordinates?)
            % * add impedance controller
            % * remove efforts if all zeros? e.g. strategy 3

        end
        
        function [] = send(this, varargin)
            % SEND sends the current command state to the module group
            
            % Setup command struct
            this.cmd.position = this.state.cmdPos;
            this.cmd.velocity = this.state.cmdVel;
            this.cmd.effort = this.state.cmdEffort;
            
            % Send state to module
            this.group.send(this.cmd, varargin{:});
        end
        
    end
    
    % Utility methods used internally
    methods(Access = private)
        
        function gravCompEfforts = getGravCompEfforts(this, fbk)
            
            % Find gravity vector by looking at orientation of first joint
            q = [ fbk.orientationW(1), ...
                fbk.orientationX(1), ...
                fbk.orientationY(1), ...
                fbk.orientationZ(1) ];
            baseRotMat = HebiUtils.quat2rotMat(q);
            gravityVec = -baseRotMat(3,1:3);
            
            % TODO: account for base frame rotation. Usually is eye(4)
            
            % Compensate for gravity
            gravCompEfforts = this.kin.getGravCompEfforts(fbk.position, gravityVec);
            
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

