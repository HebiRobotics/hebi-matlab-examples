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
    
    properties(Access = public)
        enableCommands logical = true; % position / velocities / dynamics comp
        enableDynamicsComp logical = true; % efforts to compensate for joint accelerations
        enableGravComp logical = true; % efforts to compensate for gravitational accelerations
        gripper HebiGripper = HebiGripper(HebiUtils.newImitationGroup(1)); % auxiliary device
    end
    
    properties(SetAccess = private)
        group HebiGroup;
        kin HebiKinematics;
        trajGen HebiTrajectoryGenerator;
        traj HebiTrajectory;
    end
    
    properties(Access = private)
        trajStartTime double = 0;
        prevState struct = [];
        nZeros double;
    end
    
    methods
        
        function this = HebiArm(varargin)
            this.group = varargin{1};
            this.kin = varargin{2};
            
            this.trajGen = HebiTrajectoryGenerator(this.kin);
            this.trajGen.setMinDuration(1.00); % min move time for 'small' movements (default 1.0)
            this.trajGen.setSpeedFactor(0.75); % slow down to safer speed (default = 1.0)
            
            this.nZeros = zeros(1, this.kin.getNumDoF);
        end
        
        function [] = initialize(this)
            % Initializes with soft-start to current feedback
            % maybe also a moveHome()? setHome()?
            
            fbk = this.group.getNextFeedbackFull();
            this.traj = this.trajGen.newJointMove(...
                [fbk.position; fbk.position], ...
                'time', [0 1]);
            this.update(); % init state
            
        end

        function [] = setGoal(this, varargin)
            %arm.setGoal('positions', pos, 'velocities', vel, 'accelerations', accel, 'time', time); 
            % Calculates trajectory from current pos (fbk) to target
            % same as newJointMove, but includes current state? [cmdPos pos] => traj generator
            
            if isempty(this.prevState)
                error('trajectory has not yet been initialized');
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
            defaultConstraints = goal.Positions;
            defaultConstraints(:) = nan;
            defaultConstraints(end,:) = 0;
            if isempty(goal.Velocities)
                goal.Velocities = defaultConstraints;
            end
            if isempty(goal.Accelerations)
                goal.Accelerations = defaultConstraints;
            end
            
            % Create trajectory starting from last known state 
            % (TODO: timing / duration?)
            this.traj = this.trajGen.newJointMove(...
                [this.prevState.cmdPos; goal.Positions], ...
                'Velocities', [this.prevState.cmdVel; goal.Velocities], ...
                'Accelerations', [this.prevState.cmdAccel; goal.Accelerations]);
            this.trajStartTime = this.prevState.time;
            
        end
        
        function value = isAtGoal(this)
            value = this.getProgress() >= 1;
        end
        
        function [progress] = getProgress(this)
            % returns normalized [0-1] time of where trajectory is
            % currently at
            if isempty(this.traj) || isempty(this.prevState)
                error('trajectory not initialized');
            end
            
            t = this.prevState.time-this.trajStartTime;
            progress = min(t / this.traj.getDuration(),1);
            
        end
        
        function [cmd, state] = update(this)
            % reads feedback, updates state, calculates commands
            
            % Read feedback
            fbk = this.group.getNextFeedbackFull();
            time = fbk.time;
            
            % Initialize efforts
            numDoF = this.kin.getNumDoF();
            cmdPos = [];
            cmdVel = [];
            cmdAccel = [];
            cmdEffort = zeros(1, numDoF);
            
            % Compensate for accelerations due to gravity
            if this.enableGravComp
                cmdEffort = cmdEffort + this.getGravCompEfforts(fbk);
            end
            
            % Add pos/vel/accel commands
            if this.enableCommands && ~isempty(this.traj)
                
                % Evaluate trajectory state
                t = min(time - this.trajStartTime, this.traj.getDuration());
                [cmdPos, cmdVel, cmdAccel] = this.traj.getState(t);
                
                % Compensate for joint accelerations
                if this.enableDynamicsComp
                    cmdEffort = cmdEffort + this.kin.getDynamicCompEfforts(...
                        fbk.position, cmdPos, cmdVel, cmdAccel);
                end
                
            end
            
            % Setup command struct
            cmd = CommandStruct(); % TODO: reuse somehow?
            cmd.position = cmdPos;
            cmd.velocity = cmdVel;
            cmd.effort = cmdEffort;
            
            % Populate state feedback
            state = struct();
            state.fbk = fbk;
            state.time = time;
            state.cmdPos = cmdPos;
            state.cmdVel = cmdVel;
            state.cmdAccel = cmdAccel;
            state.cmdEffort = cmdEffort;
            
            % TODO: add FK, Jacobians, End-Effector XYZ / Wrench, etc.
            this.prevState = state;
            
            % TODO: --> user code
            % * add effort offsets for e.g. spring.
            % * add efforts to add an end effector wrench (local or world coordinates?)
            % * add impedance controller
            % * remove efforts if all zeros? e.g. strategy 3

        end
        
        function [] = send(this, cmd)
            % refreshes group commands of arm and gripper
           this.group.send(cmd); 
           this.gripper.setState(this.gripper.state);
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

