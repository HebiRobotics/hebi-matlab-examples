classdef (Sealed) HebiTrajectoryGenerator
    % HebiTrajectoryGenerator provides support for creating trajectories
    %
    % For series elastic actuators such as the X-series modules it is
    % important to command smooth trajectories to avoid oscillations
    % induced by the spring element. Additionally, it helps significantly
    % to not only command positions, but also velocities and if possible
    % efforts (torques). 
    %
    %   HebiTrajectoryGenerator Methods (non-blocking):
    %      setAlgorithm      - sets the trajectory algorithm
    %      setMinDuration    - sets the minimum duration
    %      setSpeedFactor    - sets the speed factor
    %      newJointMove      - creates a trajectory through joint waypoints
    %      newLinearMove     - creates a trajectory through xyz waypoints
    %
    %   HebiTrajectoryGenerator Methods (blocking):
    %      moveJoint         - creates and executes joint trajectory
    %      moveLinear        - creates and executes xyz trajectory
    %      executeTrajectory - executes a given trajectory
    %
    % Note that trajectories can technically be calculated without
    % knowledge of the kinematic configuration. This API requires the
    % HebiKinematics object mainly in order to access joint meta-data such 
    % as velocity limits. A full model of the system is only required for
    % converting accelerations to efforts (torques) as well as 'linear'
    % movements that move the end-effector in world coordinates. 
    %
    % Thus, even if your robot can not be expressed as a serial chain 
    % (e.g. delta arm), you may still use this API to calculate  
    % trajectories by only specifying the system's joints.
    %
    % Execution of a trajectory additionally requires a HebiGroup in order
    % to communicate with hardware.
    %
    %   Example
    %      % Setup kinematics and create trajectory generator
    %      kin = HebiKinematics();
    %      kin.addBody('X5-4');
    %      kin.addBody('X5-Link', 'ext', 0.2875, 'twist', 0, 'mass', 0.318);
    %      kin.addBody('X5-1');
    %      kin.addBody('X5-Link', 'ext', 0.3678, 'twist', 0, 'mass', 0.276);
    %      kin.addBody('X5-1');
    %      trajGen = HebiTrajectoryGenerator(kin);
    %
    %      % Move between a set of waypoints using blocking calls
    %      numWaypoints = 20;
    %      positions = rand(numWaypoints, kin.getNumDoF);
    %      for i = 2:numWaypoints
    %           start = positions(i-1, :);
    %           finish = positions(i, :);
    %           trajGen.moveJoint(group, [start; finish]);
    %      end
    %
    %   Example
    %      % Generate a non-blocking trajectory
    %      trajGen = HebiTrajectoryGenerator(kin);
    %      trajectory = trajGen.newJointMove([start; finish]);
    %
    %      % Visualize points along the trajectory
    %      t = 0:0.001:trajectory.getDuration();
    %      [pos, vel, accel] = trajectory.getState(t);
    %      plot(t, pos);
    %
    %      % Execute trajectory open-loop in position and velocity
    %      cmd = CommandStruct();
    %      t0 = tic();
    %      t = toc(t0);
    %      while t < trajectory.getDuration()
    %          t = toc(t0);
    %          [cmd.position, cmd.velocity, ~] = trajectory.getState(t);
    %          group.send(cmd);
    %          pause(0.001);
    %      end
    %
    %   See also HebiKinematics, HebiLookup, HebiGroup, newJointMove
    
    %   Copyright 2014-2017 HEBI Robotics, LLC.
    
    %% API Methods
    methods(Access = public)
        
         function out = getAlgorithm(this, varargin)
            % getAlgorithm gets the trajectory algorithm
            out = getAlgorithm(this.obj, varargin{:});
        end
        
        function this = setAlgorithm(this, varargin)
            % setAlgorithm sets the trajectory algorithm
            %
            %   Currently implemented algorithms are
            %       
            %       'UnconstrainedQp'   (default)
            %       'MinJerkPhase'      
            %
            %   'UnconstrainedQp' creates advanced minimum jerk 
            %   trajectories that support user specified velocity and 
            %   acceleration constraints at any point (free constraints 
            %   are represented by nans).
            %
            %   'MinJerkPhase' is an older trajectory algorithm that
            %   requires fully specified position waypoints and does not
            %   support user specified constraints. We consider this
            %   algorithm deprecated and recommend not using it anymore.
            % 
            %   See also HebiTrajectoryGenerator, newJointMove
            setAlgorithm(this.obj, varargin{:});
        end
        
        function out = getMinDuration(this, varargin)
            % getMinDuration returns the minimum auto-determined duration
            out = getMinDuration(this.obj, varargin{:});
        end
        
        function this = setMinDuration(this, varargin)
            % setMinDuration sets the minimum auto-determined duration
            setMinDuration(this.obj, varargin{:});
        end
        
        function out = getSpeedFactor(this, varargin)
            % getSpeedFactor gets the speed factor
            out = getSpeedFactor(this.obj, varargin{:});
        end
        
        function this = setSpeedFactor(this, varargin)
            % setSpeedFactor sets the speed factor
            %
            %   The speed factor reduces the speed of all trajectories
            %   to the specified multiplier. For example, a speed factor
            %   of 0.5 would turn a 5 second trajectory into a 10 second
            %   trajectory.
            %
            %   This factor gets applied to all trajectories, even
            %   ones that have user specified time.
            %
            %   Example:
            %       % double the duration of all trajectories
            %       trajGen = HebiTrajectoryGenerator(kin);
            %       trajGen.setSpeedFactor(0.5);
            %
            %   Values above 1 are supported, but may result in
            %   trajectories that exceed the joint limits.
            %
            %   See also getSpeedFactor, newJointMove
            setSpeedFactor(this.obj, varargin{:});
        end
        
        function trajectory = newJointMove(this, varargin)
            % newJointMove creates a trajectory through joint waypoints
            %
            % Arguments:
            %
            %   Positions        - N x numJoints matrix
            %
            % Parameters:
            %
            %   Note that some trajectory algorithms may not support all of
            %   the following parameters.
            %
            %   'Duration' [s] is a scalar that sets the desired duration
            %   at which the trajectory should be completed. This
            %   overwrites any automatically determined duration, so the
            %   resulting trajectory may not be technically feasible.
            %
            %   'Time' [s] is a vector of time constraints for each
            %   waypoint. This overwrites any automatically determined
            %   duration, so the resulting trajectory may not be
            %   technically feasible. The vector needs to start at zero, 
            %   be monotonically increasing, and not contain nan or inf.
            %
            %   'Velocities' [rad/s] is a matrix of velocity constraints
            %   for each waypoint. Unspecified values ('nan') will be
            %   determined automatically. If velocities are not specified,
            %   the default assumes zero start and end conditions.
            %   For a waypoint for 4 joints over 5 waypoints this would be
            %   equivalent to the following
            %       
            %       defaultVelocities = [
            %            0   0   0   0 
            %           nan nan nan nan
            %           nan nan nan nan
            %           nan nan nan nan
            %            0   0   0   0 ];
            %   
            %   'Accelerations' [rad/s^2] is a matrix of acceleration 
            %   constraints for each waypoint. Unspecified values ('nan') 
            %   will be determined automatically. If accelerations are not 
            %   specified, the default is the same as for velocities, i.e.,
            %   zero start and end conditions.
            %
            %   Example:
            %       % Create trajectory with automated time scaling
            %       positions = rand(10, kin.getNumDoF());
            %       trajectory = trajGen.newJointMove(positions);
            %
            %   Example:
            %       % Create a trajectory that goes through 10 waypoints
            %       % in 4 seconds.
            %       positions = rand(10, kin.getNumDoF());
            %       trajectory = trajGen.newJointMove(positions, ...
            %           'Duration', 4.0);
            %
            %   Example:
            %       % Visualize the trajectory
            %       positions = rand(10, kin.getNumDoF());
            %       trajectory = trajGen.newJointMove(positions);
            %       t = 0:0.001:trajectory.getDuration();
            %       [pos,vel,accel] = trajectory.getState(t);
            %       plot(t, pos);
            %
            %   Example:
            %       % Create trajectory
            %       positions = rand(10, kin.getNumDoF());
            %       trajectory = trajGen.newJointMove(positions);
            %
            %       % Manually execute position/velocity/effort trajectory
            %       cmd = CommandStruct();
            %       t0 = tic();
            %       t = toc(t0);
            %       while t < trajectory.getDuration()
            %           fbk = group.getNextFeedback();
            %
            %           % get state at current time interval
            %           t = toc(t0);
            %           [pos, vel, accel] = trajectory.getState(t);
            %
            %           % compensate for gravity
            %           gravityVec = -[fbk.accelX(1) fbk.accelY(1) fbk.accelZ(1)];
            %           gravCompEffort = kin.getGravCompEfforts(fbk.position, gravityVec);
            %
            %           % compensate for accelerations
            %           accelCompEffort = kin.getDynamicCompEfforts(...
            %               fbk.position, ... % used for jacobian
            %               pos, vel, accel);
            %
            %           % send to hardware
            %           cmd.position = pos;
            %           cmd.velocity = vel;
            %           cmd.effort = gravCompEffort + accelCompEffort;
            %           group.send(cmd);
            %
            %      end
            %
            %   Example:
            %       % Setup trajectory generator for a single joint
            %       kin = HebiKinematics();
            %       kin.addBody('X5-1');
            %       trajGen = HebiTrajectoryGenerator(kin);
            %       trajGen.setSpeedFactor(1);
            %       trajGen.setAlgorithm('UnconstrainedQp');
            %
            %       % Create single joint trajectory with constraints
            %       positions     = [ 0  1   3   3   1   0 ]';
            %       velocities    = [ 0 nan  0   0  nan  0 ]';
            %       accelerations = [ 0 nan nan nan nan  0 ]';
            %       time          = [ 0  1   2   3   4   5 ]';
            %
            %       trajectory = trajGen.newJointMove( positions, ...
            %           'Velocities', velocities, ...
            %           'Accelerations', accelerations, ...
            %           'Time', time );
            %
            %       % Visualize result
            %       t = 0:0.01:trajectory.getDuration();
            %       [p,v,a] = trajectory.getState(t);
            %       subplot(3,1,1);
            %       plot(t,p);
            %       title('position')
            %       subplot(3,1,2);
            %       plot(t,v);
            %       title('velocity')
            %       subplot(3,1,3);
            %       plot(t,a);
            %       title('acceleration')
            %       
            %   See also HebiTrajectoryGenerator, moveJoint, newJointMove,
            %   executeTrajectory
            trajectory = HebiTrajectory(newJointMove(this.obj, varargin{:}));
        end
        
        function trajectory = newLinearMove(this, positions, varargin)
            % newLinearMove creates a trajectory through xyz waypoints
            %
            %   This method works similar to newJointMove with the main
            %   difference being that the resulting trajectory represents
            %   an approximate straight line in cartesian workspace
            %   coordinates.
            %
            %   At the moment this method supports only exactly two
            %   waypoints per call. This method also currently only
            %   supports the following parameters.
            %
            %       'Duration'  - time to completion [s]
            %
            %   'Velocities', 'Accelerations', and 'Time' constraints are
            %   currently not supported.
            %
            %   Please see the newJointMove documentation for more information. 
            %
            %   See also HebiTrajectoryGenerator, newJointMove, moveLinear,
            %   executeTrajectory
            if size(positions,1) > 2
                error('Missing support for more than two waypoints.');
            end
            
            if ~exist('vrrotmat2vec','file')
                error('Requires Simulink 3D Animation toolbox. Dependency will be removed soon.');
            end
            
            if ~strcmp('MinJerkPhase', this.getAlgorithm)
               error('newLinearMove currently only supports MinJerkPhase algorithm'); 
            end
            
            % Interpolate between XYZ endpoint positions
            T0 = this.kin.getForwardKinematicsEndEffector(positions(1,:));
            Tn = this.kin.getForwardKinematicsEndEffector(positions(end,:));
            
            xyzPositions = nan(3,2);
            xyzPositions(:,1) = T0(1:3,4);
            xyzPositions(:,2) = Tn(1:3,4);
            regPhase = [ 0 1 ];
            
            upSamplePoints = 20;
            upSampledPhase = linspace(0,1,upSamplePoints);
            upSampledXYZ = interp1(regPhase, xyzPositions', upSampledPhase, 'linear' );
            upSampledAngles = interp1(regPhase, positions, upSampledPhase, 'linear' );
            
            % Disable so3 IK on 'large' rotations
            diffDCM = T0(1:3,1:3) * Tn(1:3,1:3)';
            axisAngle = vrrotmat2vec(diffDCM);
            looseIK = abs(axisAngle(4)) > pi/4;
            if looseIK
                display('Large Orientation Change. Doing "Loose" IK.');
            end
            
            % Get the joint angles for each waypoint
            IKAngles = nan(upSamplePoints, this.kin.getNumDoF());
            for i=1:upSamplePoints
                
                initAngles = upSampledAngles(i,:);
                
                if looseIK
                    IKAngles(i,:) = getInverseKinematics( this.kin, ...
                        'xyz', upSampledXYZ(i,:)', ...
                        'initial', initAngles );
                else
                    cmdDCM = this.kin.getForwardKinematicsEndEffector(initAngles);
                    IKAngles(i,:) = getInverseKinematics( this.kin, ...
                        'xyz', upSampledXYZ(i,:)', ...
                        'so3', cmdDCM, ...
                        'initial', initAngles );
                end
                
            end
            
            % Construct joint move trajectory over interpolated angles
            trajectory = this.newJointMove(IKAngles, varargin{:});
        end
        
        function [] = moveJoint(this, group, positions, varargin)
            % moveJoint creates and executes a joint trajectory
            %
            %   This method is a convenience wrapper that is equivalent
            %   to manually calling the following:
            %
            %      traj = trajGen.newJointMove(positions);
            %      trajGen.executeTrajectory(group, traj);
            %
            %   Arguments:
            %
            %       group     - target actuators (HebiGroup)
            %       positions - N x numModules matrix of positions
            %
            %   Please check the documentation for newJointMove and
            %   executeTrajectory for information on available parameters.
            %
            %   Example
            %      % Move between waypoints with stops in between
            %      numWaypoints = 20;
            %      positions = rand(numWaypoints, kin.getNumDoF);
            %      for i = 2:numWaypoints
            %           start = positions(i-1, :);
            %           finish = positions(i, :);
            %           trajGen.moveJoint(group, [start; finish]);
            %      end
            %
            %   Example
            %      % Move through multiple waypoints at once
            %      numWaypoints = 20;
            %      positions = rand(numWaypoints, kin.getNumDoF);
            %      trajGen.moveJoint(group, positions);
            %
            %   See also HebiTrajectoryGenerator, newJointMove,
            %   executeTrajectory
            traj = this.newJointMove(positions);
            this.executeTrajectory(group, traj, varargin{:});
        end
        
        function [] = moveLinear(this, group, positions, varargin)
            % moveLinear moves between waypoints in straight lines
            %
            %   This method works the same as moveJoint with the only
            %   difference being that the resulting trajectory represents
            %   an approximate straight line in cartesian workspace
            %   coordinates.
            %
            %   This method is a convenience wrapper that is equivalent
            %   to manually calling the following:
            %
            %      traj = trajGen.newLinearMove(positions);
            %      trajGen.executeTrajectory(group, traj);
            %
            %   Please refer to the documentation of newJointMove and
            %   executeTrajectory for more information.
            %
            %   See also HebiTrajectoryGenerator, moveJoint, newJointMove,
            %   executeTrajectory
            traj = this.newLinearMove(positions);
            this.executeTrajectory(group, traj, varargin{:});
        end
        
        function [] = executeTrajectory(this, varargin)
            % executeTrajectory executes a trajectory and returns when done
            %
            %   Arguments:
            %
            %       group      - target actuators (HebiGroup)
            %       trajectory - executable trajectory (HebiTrajectory)
            %
            %   Parameters:
            %
            %   'GravityVec' enables gravity compensation. It expects a
            %   [3 x 1] vector of the direction of gravity in the base
            %   frame.  Note that this direction vector is not required to
            %   be unit length, and that gravitational acceleration is
            %   assumed to be 9.81 m/s^2.
            %
            %   'EnableDynamicsComp' [true/false] enables feeding forward
            %   efforts to compensate for joint accelerations. Dynamics
            %   compensation is turned off by default and should typically
            %   only be turned on if Gravity compensation is activated as
            %   well.
            %
            %   'EffortOffset' sets effort offsets that get applied if
            %   gravity compensation or dynamics compensation is turned on.
            %   This can be used to account for physical features (e.g. a
            %   gas spring) that are not captured by other parts of the
            %   system.
            %
            %   'Callback' is a user supplied function that gets called
            %   once per iteration. This offers a way for users to, e.g.,
            %   detect impacts or add impedance control. The function must
            %   accept 3 arguments (time,fbk,cmd) and return a
            %   CommandStruct. For example:
            %
            %      % Callback with no change to the CommandStruct
            %       callback = @(time, fbk, cmd) cmd;
            %
            %   Example
            %       % Create trajectory
            %       trajGen = TrajectoryGenerator(kin);
            %       positions = [0 0; 1 1]
            %       trajectory = trajGen.newJointMove(positions);
            %
            %       % Simple execution
            %       trajGen.executeTrajectory(group, trajectory);
            %
            %       % Execution with full options
            %       gravityVec = -[fbk.accelX(1) fbk.accelY(1) fbk.accelZ(1)];
            %       trajGen.executeTrajectory(group, trajectory, ...
            %           'GravityVec', gravityVec, ...
            %           'DynamicsComp', true, ...
            %           'EffortOffset', zeros(1, kin.getNumDoF), ...
            %           'Callback', @(time, fbk, cmd) cmd);
            %
            %   See also HebiTrajectoryGenerator, newJointMove, moveJoint,
            %   newLinearMove
            
            % Setup parser
            parser = inputParser();
            parser.addRequired('Group', @(a) isa(a, 'HebiGroup'));
            parser.addRequired('Trajectory', @(a) isa(a, 'HebiTrajectory'));
            parser.addParameter('GravityVec', [], @(a) isempty(a) || isvector(a));
            parser.addParameter('EnableDynamicsComp', false, @(a) islogical(a));
            parser.addParameter('Callback', []); % default to no change
            parser.addParameter('EffortOffset', [], @(a) isvector(a));
            parser.CaseSensitive = false;
            parser.parse(varargin{:});
            p = parser.Results;
            
            % Simplify names
            group = p.Group;
            traj = p.Trajectory;
            
            % Execute trajectory and return when done
            tmpFbk = group.getNextFeedback();
            duration = traj.getDuration();
            cmd = CommandStruct();
            t0 = tic();
            t = toc(t0);
            while t < duration
                
                fbk = group.getNextFeedback(tmpFbk);
                jacobianPosition = fbk.position;
                
                t = toc(t0);
                [pos, vel, accel] = traj.getState(t);
                
                % Convert accelerations to efforts
                effort = zeros(size(pos));
                useEfforts = false;
                if p.EnableDynamicsComp
                    accelCompEffort = this.kin.getDynamicCompEfforts(...
                        jacobianPosition, ...
                        pos, ...
                        vel, ...
                        accel);
                    effort = effort + accelCompEffort;
                    useEfforts = true;
                end
                
                % Account for gravity if applicable
                if ~isempty(p.GravityVec)
                    gravCompEffort = this.kin.getGravCompEfforts(...
                        jacobianPosition, ...
                        p.GravityVec);
                    effort = effort + gravCompEffort;
                    useEfforts = true;
                end
                
                % Offset for springs etc., if applicable
                if useEfforts && ~isempty(p.EffortOffset)
                    effort = effort + p.EffortOffset;
                end
                
                % Disable effort control if not needed
                if ~useEfforts
                    effort = nan(size(pos));
                end
                
                % Command to module
                cmd.position = pos;
                cmd.velocity = vel;
                cmd.effort = effort;
                
                % Add user callback, e.g.,
                if ~isempty(p.Callback)
                    cmd = p.Callback(t, fbk, cmd);
                end
                
                % Send Update
                group.send(cmd);
                
            end
        end
        
        function this = HebiTrajectoryGenerator(kin)
            % Provides methods to generate trajectories
            if ~isa(kin, 'HebiKinematics')
                error('Expected HebiKinematics');
            end
            this.kin = kin;
            
            this.obj = javaObject(HebiTrajectoryGenerator.className);
            setKinematics(this.obj, kin.obj);
            setAlgorithm(this.obj, this.config.defaultAlgorithm);
            setMinDuration(this.obj, this.config.defaultMinDuration);
            setSpeedFactor(this.obj, this.config.defaultSpeedFactor);
        end
        
    end
    
    %% Library load
    properties(Constant, Access = private, Hidden = true)
        className = hebi_load('HebiTrajectoryGenerator');
        config = hebi_config('HebiTrajectoryGenerator');
    end
    
    %% State Properties
    properties(Access = private, Hidden = true)
        kin;
        obj;
    end
    
    % Hidden Methods
    methods(Access = public, Hidden = true)
        function disp(this)
            disp(this.obj);
        end
    end
    
end

