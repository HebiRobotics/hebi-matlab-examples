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
        pluginList cell; % plugins as a list
        plugins struct = struct(); % plugin lookup by names
    end
    
    properties(Access = private)
        cmd = CommandStruct();
        trajStartTime double = 0;
        nZeros double;
        aux double;
        auxTime double;
    end

    % Accessors to keep plugin list and struct in sync
    methods
        function plugins = get.plugins(this)
            % plugin lookup by names
            plugins = struct();
            for i=1:length(this.pluginList)
                % TODO: duplicate names -> pick last or first? return array?
                plugin = this.pluginList{i};
                plugins.(plugin.name) = plugin;
            end
        end
        function [] = set.plugins(this, input)
            % Return the plugin list as a struct
            fields = fieldnames(input);
            this.pluginList = cell(length(fields),1);
            for i = 1:length(fields)
                this.pluginList{i} = input.(fields{i});
            end
        end
    end

    methods(Static)
        function [arm, config] = createFromConfig(configOrPath)
            % createFromConfig creates a HebiArm according to the config
            %
            % The config can be a config struct or a path to the file. The
            % config is also returned to work as a shortcut for HebiUtils.loadRobotConfig
            %
            % See also HebiUtils.loadRobotConfig

            % Support config structs or file paths
            if isa(configOrPath, 'struct')
                config = configOrPath;
            elseif exist(configOrPath, 'file')
                config = HebiUtils.loadRobotConfig(configOrPath);
            else
                error('expected config struct or path to existing confg file');
            end

            % Create backing group comms
            group = HebiLookup.newGroupFromNames(config.families, config.names);

            % Setup default gains
            if isfield(config.gains, 'default')
                gains = HebiUtils.loadGains(config.gains.default);
                HebiUtils.sendWithRetry(group, 'gains', gains);
            end

            % Setup arm with kinematics and plugins
            kin = HebiUtils.loadHRDF(config.hrdf);
            arm = HebiArm(group, kin);
            arm.plugins = HebiArmPlugin.createFromConfigMap(config.plugins);

            % Initialize state
            arm.update();
        end
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
            % arm.setGoal(pos, 'velocities', vel, 'accelerations', accel, 'time', time); 
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
            newState.outputFrames = this.kin.getForwardKinematics('output', fbkPos);
            newState.T_endEffector = newState.outputFrames(:,:,end);
            newState.J_endEffector = this.kin.getJacobianEndEffector(fbkPos);
            
            % Add pos/vel/accel commands
            newState.cmdAux = [];
            if ~isempty(this.traj)
                
                % Evaluate trajectory state
                t = min(newState.time - this.trajStartTime, this.traj.getDuration());
                [ newState.cmdPos,  newState.cmdVel,  newState.cmdAccel] = this.traj.getState(t);
                
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
            
            % Efforts (grav comp, dynamic comp, etc.) are added by plugins
            newState.cmdEffort = zeros(1, newState.numDoF);

            % Call plugins (FK, Jacobians, End-Effector XYZ, etc.)
            this.state = newState;
            for i=1:length(this.pluginList)
                plugin = this.pluginList{i};
                if plugin.enabled % Optional: check if the plugin is enabled
                    plugin.update(this);
                end
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

        function plugin = getPluginByType(this, type)
            % getPluginByType Retrieves a plugin by its plugin type
            %
            % Example:
            %   plugin = arm.getPluginByType('GravityCompensationEffort');
            
            pluginFields = fieldnames(this.plugins);
            for i = 1:length(pluginFields)
                plugin = this.plugins.(pluginFields{i});
                if strcmp(plugin.type, type)
                    return;
                end
            end
            error('Plugin of type "%s" not found.', type);
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

