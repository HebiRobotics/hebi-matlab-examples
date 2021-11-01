classdef HebiArm < handle
    %HEBIARM is a higher-level API for working with serial configurations
    %
    %   HebiArm simplifies many of the tasks commonly encountered when
    %   with arm-like configurations such as trajectory replanning,
    %   acceleration compensation, and keeping track of auxiliary state 
    %   such as a gripper.
    %
    %   It also features a plugin mechanism to enable optional features
    %   such as offsetting efforts when working with external springs, or
    %   adding impedance control.
    %
    %   Example
    %      % Setup the arm
    %      group = HebiLookup.newGroupFromNames('Arm', 'J*');
    %      kin = HebiUtils.loadHRDF('A-2085-06.hrdf');
    %      arm = HebiArm(group, kin);
    %
    %      % Move to home position
    %      arm.update();
    %      arm.setGoal(homePosition);
    %      while ~arm.isAtGoal()
    %          arm.update();
    %          arm.send();
    %      end
    % 
    % See also HebiArmPlugin
    
    properties(SetAccess = private)
        group HebiGroup;
        kin HebiKinematics;
        trajGen HebiTrajectoryGenerator;
        traj;
    end
    
    properties(Access = public)
        state struct = [];
        plugins cell = {};
    end
    
    properties(Access = private)
        cmd = CommandStruct();
        trajStartTime double = 0;
        nZeros double;
        aux double;
        auxTime double;
    end
    
    methods
        
        function this = HebiArm(varargin)
            % HebiArm requires a HebiGroup of devices and a corresponding
            % HebiKinematics object.
            if length(varargin) ~= 2
               error('expected 2 arguments: group, kinematics'); 
            end
            this.group = varargin{1};
            this.kin = varargin{2};
            
            this.trajGen = HebiTrajectoryGenerator(this.kin);
            this.nZeros = zeros(1, this.kin.getNumDoF);
        end
        
        function [] = clearGoal(this)
            % CLEARGOAL cancels any active goal, returning to a
            % "weightless" state that does not actively command position
            % or velocity
            this.traj = [];
        end

        function [] = setGoal(this, varargin)
            %arm.setGoal('positions', pos, 'velocities', vel, 'accelerations', accel, 'time', time); 
            % Calculates trajectory from current pos (fbk) to target
            % same as newJointMove, but includes current state? [cmdPos pos] => traj generator
            
            if isempty(this.state)
               error('Internal state has not been initialized. Please call update() before calling setGoal()'); 
            end
            
            % Recalculates the active trajectory from the last commanded
            % state to the set goal waypoints
            parser = inputParser();
            parser.addRequired('Positions');
            parser.addOptional('Velocities', []);
            parser.addOptional('Accelerations', []);
            parser.addOptional('Aux', []);
            parser.addOptional('Time', []);
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
            
            timeArgs = {};
            if ~isempty(goal.Time)
                timeArgs = {'Time', [0; goal.Time(:)]};
            end
            
            % Create trajectory starting from last known state 
            this.traj = this.trajGen.newJointMove(...
                [cmdPos; goal.Positions], ...
                'Velocities', [cmdVel; goal.Velocities], ...
                'Accelerations', [cmdAccel; goal.Accelerations], ...
                timeArgs{:});
            this.trajStartTime = startTime;
            
            if isempty(goal.Aux)
               this.aux = [];
               this.auxTime = [];
            else
               initialAux = goal.Aux(1,:) * nan; % TODO: improve when known?
               this.aux = [initialAux; goal.Aux];
               this.auxTime = this.traj.getWaypointTime();
               if size(this.aux, 1) ~= length(this.auxTime)
                   error(['Expected Aux: ' num2str(length(this.auxTime)-1) 'xM matrix']);
               end
            end
            
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
            
            % Add commonly derived state
            fbkPos = newState.fbk.position;
            newState.T_endEffector = this.kin.getForwardKinematicsEndEffector(fbkPos);
            newState.J_endEffector = this.kin.getJacobianEndEffector(fbkPos);
            
            % Always compensate for accelerations due to gravity
            newState.cmdEffort = this.getGravCompEfforts(newState.fbk);
            
            % Add pos/vel/accel commands
            newState.cmdAux = [];
            if ~isempty(this.traj)
                
                % Evaluate trajectory state
                t = min(newState.time - this.trajStartTime, this.traj.getDuration());
                [ newState.cmdPos,  newState.cmdVel,  newState.cmdAccel] = this.traj.getState(t);
                
                % Compensate for joint accelerations
                newState.cmdEffort = this.kin.getDynamicCompEfforts(...
                    fbkPos, ...
                    newState.cmdPos, ...
                    newState.cmdVel, ...
                    newState.cmdAccel) + newState.cmdEffort;
                
                % Add aux (e.g. gripper state)
                if ~isempty(this.aux)
                    auxRow = find(t >= this.auxTime, 1, 'last' );
                    newState.cmdAux = this.aux(auxRow, :);
                end
                
                newState.trajTime = t;
                
            else
                newState.cmdPos = [];
                newState.cmdVel = [];
                newState.cmdAccel = [];
                newState.trajTime = [];
            end
            
            % Add time delta since last feedback, if available
            newState.dt = 0;
            if ~isempty(this.state)
                dt = (newState.time - this.state.time);  
                
                % Avoid freak-outs related to users restarting
                % scripts without creating a new instance
                if dt > 0 && dt < 1
                   newState.dt = dt; 
                end
            end
            
            % Call plugins (FK, Jacobians, End-Effector XYZ, etc.)
            this.state = newState;
            for i=1:length(this.plugins)
                this.plugins{i}.update(this);
            end            

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
    methods(Access = protected)
        
        function gravCompEfforts = getGravCompEfforts(this, fbk)
            
            % Find gravity vector by looking at orientation of first joint
            q = [ fbk.orientationW(1), ...
                  fbk.orientationX(1), ...
                  fbk.orientationY(1), ...
                  fbk.orientationZ(1) ];
              
            if any(isnan(q))
                % If the group does not provide orientation feedback, we assume
                % that gravity points 'down' in the base frame (-Z Axis).
                warning('No orientation feedback available. Assuming gravity points down.');
                gravityVec = [0; 0; -1];
            else
                % The orientation feedback is in the IMU frame, so we 
                % need to first rotate it into the world frame.
                baseRotMat = HebiUtils.quat2rotMat(q);
                imuFrame = this.kin.getFirstJointFrame();
                gravityVec = imuFrame(1:3,1:3) * (-baseRotMat(3,1:3)');
            end
            
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

