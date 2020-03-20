classdef (Sealed) HebiTrajectoryGenerator
    % HebiTrajectoryGenerator provides support for creating trajectories
    %
    % For series elastic actuators such as the X-series modules it is
    % important to command smooth trajectories to avoid oscillations
    % induced by their mechanical compliance. Additionally, it helps to not 
    % only command positions, but also velocities and efforts (torques). 
    %
    %   HebiTrajectoryGenerator Methods (non-blocking):
    %
    %      newJointMove   - Creates a trajectory through joint waypoints
    %                       that does not make use of kinematics.
    %
    %      newLinearMove  - Creates a trajectory through xyz waypoints
    %                       that uses knowledge of kinematics to travel in
    %                       straight lines in workspace.
    %
    %      setMinDuration - Sets the minimum duration that gets used when
    %                       relying on the trajectory generator's internal
    %                       timing heuristic.
    %      getMinDuration - Returns the currently set minimum trajectory
    %                       duration.
    %
    %      setSpeedFactor - Sets a scalar that adjusts the speed of all
    %                       trajectories. A value of 1.0 will run at full
    %                       speed, and smaller values proportionally
    %                       slow down the timing of the trajectory. This
    %                       applies to both the internal timing heuristic
    %                       as well as user supplied time vectors.
    %      getSpeedFactor - Returns currently set speed factor.
    %
    %      setAlgorithm   - Allows use of legacy trajectory algorithms.
    %      getAlgorithm   - Returns the active trajectory algorithm.
    %
    %   HebiTrajectoryGenerator Methods (blocking):
    %
    %      executeTrajectory - Executes a trajectory that is generated with
    %                          either newJointMove or newLinearMove.  This
    %                          method provides more control on how the
    %                          trajectory is setup, allowing multiple
    %                          intermediate waypoints and additional
    %                          constraints such as user-defined waypoint
    %                          timing and non-zero waypoint velocities and
    %                          accelerations.                   
    %
    % Note that the following parts of the API require additional knowledge 
    % about the hardware:
    %
    %   - The internal heuristic to automatically determine an appropriate
    %     time vector requires knowledge of joint velocity limits
    %
    %   - Linear movements (newLinearMove) require a fully 
    %     specified kinematic model (HebiKinematics object) for IK
    %
    %   - executeTrajectory requires a fully specified kinematic model
    %     (HebiKinematics object) if gravity or acceleration compensation 
    %     are enabled.
    %
    % Execution of a trajectory using the simplified 'blocking' API
    % additionally requires a HebiGroup in order to communicate with the
    % actuator hardware.
    %
    %   Example (blocking API)
    %      kin = HebiKinematics();
    %      trajGen = HebiTrajectoryGenerator();
    %
    %      % Move between a set of waypoints using blocking calls
    %      % (Assumes a group has already been created)
    %      numWaypoints = 5;
    %      timeToMove = 3; % seconds
    %
    %      positions = rand(numWaypoints, group.getNumModules);
    %      time = [0 timeToMove];
    %
    %      for i = 2:numWaypoints
    %           start = positions(i-1, :);
    %           finish = positions(i, :);
    %           trajectory = trajGen.newJointMove(...
    %               [start; finish], ...
    %               'time', time);
    %           trajGen.executeTrajectory(group, trajectory);
    %      end
    %
    %   Example (non-blocking API)
    %      kin = HebiKinematics();
    %      trajGen = HebiTrajectoryGenerator();
    %
    %      % Move between a set of waypoints using blocking calls
    %      % (Assumes a group has already been created)
    %      numWaypoints = 5;
    %      timeToMove = 3; % seconds
    %
    %      positions = rand(numWaypoints, group.getNumModules);
    %      time = [0 timeToMove];
    %
    %      % Execute trajectory open-loop in position and velocity
    %      cmd = CommandStruct();
    %
    %      for i = 2:numWaypoints
    %         start = positions(i-1, :);
    %         finish = positions(i, :);
    %         trajectory = trajGen.newJointMove(...
    %               [start; finish], ...
    %               'time', time);
    %
    %         fbk = group.getNextFeedback();
    %         t0 = fbk.time;
    %         t = 0;
    % 
    %         while t < trajectory.getDuration()
    %            fbk = group.getNextFeedback();
    %            t = fbk.time - t0;
    %            [cmd.position, cmd.velocity, ~] = trajectory.getState(t);
    %            group.send(cmd);
    %         end
    %      end
    %
    %   Example (automatic waypoint timing) 
    %      % Generate a trajectory for XY velocities of a mobile base where
    %      % speeds are limited to 1 m/s (or whatever units you are using).
    %      velocityLimits = [ -1 1;
    %                         -1 1 ];
    %      trajGen = HebiTrajectoryGenerator(velocityLimits);
    %      positions = [ 0 0;
    %                   -2 1 ];
    %      trajectory = trajGen.newJointMove(positions);
    %
    %      % Visualize points along the trajectory
    %      HebiUtils.plotTrajectory(trajectory);
    %
    %      % Execute trajectory open-loop in position and velocity
    %      cmd = CommandStruct();
    %      t0 = tic();
    %      t = toc(t0);
    %      while t < trajectory.getDuration()
    %          t = toc(t0);
    %          fbk = group.getNextFeedback();  % limits loop rate
    %          [cmd.position, cmd.velocity, ~] = trajectory.getState(t);
    %          group.send(cmd);
    %      end
    %
    %   See also HebiTrajectory, HebiKinematics, HebiLookup, HebiGroup, 
    %   newJointMove, executeTrajectory.
    
    %   Copyright 2014-2018 HEBI Robotics, Inc.
    
    %% API Methods
    methods(Access = public)
        
         function out = getAlgorithm(this, varargin)
            % getAlgorithm gets the trajectory algorithm that is currently
            % being used to generate trajectories. 
            %
            % See also setAlgorithm
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
            %   See also HebiTrajectoryGenerator, newJointMove,
            %   getAlgorithm
            setAlgorithm(this.obj, varargin{:});
        end
        
        function out = getMinDuration(this, varargin)
            % getMinDuration returns the minimum duration between two 
            % adjacent waypoints.
            %
            % See also setMinDuration
            out = getMinDuration(this.obj, varargin{:});
        end
        
        function this = setMinDuration(this, varargin)
            % setMinDuration sets the minimum duration between two adjacent 
            % waypoints that gets used when relying on the trajectory 
            % generator's internal timing heuristic.
            %
            % This prevents large accelerations when generating a trajectory
            % between positions that are close together. Minimum duration
            % is ignored when a 'Time' vector is specified, and scaled when
            % a 'Duration' total length is set when making a new trajectory.
            %
            % See also newJointMove, getMinDuration
            setMinDuration(this.obj, varargin{:});
        end
        
        function out = getSpeedFactor(this, varargin)
            % getSpeedFactor gets the speed factor that adjusts the speed
            % of all trajectories. 
            %
            % See also setSpeedFactor
            out = getSpeedFactor(this.obj, varargin{:});
        end
        
        function this = setSpeedFactor(this, varargin)
            % setSpeedFactor sets the speed factor that adjusts the speed
            % of all trajectories.
            %
            %   The speed factor reduces the speed of all trajectories
            %   to the specified multiplier. For example, a speed factor
            %   of 0.5 would turn a 5 second trajectory into a 10 second
            %   trajectory.
            %
            %   This factor gets applied to all trajectories, including
            %   ones that have user specified time.
            %
            %   Example:
            %       % double the duration of all trajectories
            %       trajGen = HebiTrajectoryGenerator(kin);
            %       trajGen.setSpeedFactor(0.5);
            %
            %   Values above 1.0 are supported, but may result in
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
            %   Positions     - [numWaypoints x numJoints] matrix
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
            %       % Setup trajectory generator for a single joint
            %       kin = HebiKinematics();
            %       kin.addBody('X5-1');
            %       velocityLimit = kin.getJointInfo().velocityLimit;
            %       trajGen = HebiTrajectoryGenerator(velocityLimit);
            %
            %   Example:
            %       % Create trajectory with automated time scaling, with
            %       % specified settings to slow down from defaults.
            %       trajGen.setMinDuration(1.0);  
            %       trajGen.setSpeedFactor(0.5);  
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
            %       HebiUtils.plotTrajectory(trajectory);
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
            %       velocityLimit = kin.getJointInfo().velocityLimit;
            %       trajGen = HebiTrajectoryGenerator(velocityLimit);
            %
            %       % Create single joint trajectory with constraints
            %       positions     = [ 0  1   3   3   1   0 ]';
            %       velocities    = [ 0 nan  0   0  nan  0 ]';
            %       accelerations = [ 0 nan nan nan nan  0 ]';
            %       waypointTimes = [ 0  1   2   3   4   5 ]';
            %
            %       trajectory = trajGen.newJointMove( positions, ...
            %           'Velocities', velocities, ...
            %           'Accelerations', accelerations, ...
            %           'Time', waypointTimes );
            %
            %       % Visualize the position/velocity/accelerations.
            %       HebiUtils.plotTrajectory(trajectory);
            %       
            %   See also HebiTrajectoryGenerator, newJointMove,
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
            %   NOTE:
            %   Because the underlying IK relies on a local optimizer, the
            %   generated trajectory and waypoints for large motions, 
            %   particularly large changes in orientation, may be poor. It 
            %   is important to check the output of newLinearMove() before
            %   sending it actual hardware.
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
            
            if isempty(this.kin)
                error(['To use newLinearMove() HebiTrajectoryGenerator needs ' ...
                      'to be initialized with a HebiKinematics object (for IK).']);          
            end
            
            numDoF = this.kin.getNumDoF();
            
            % Make sure we have at least 2 joints.  3-DoF IK should work on
            % a 2-DoF arm.
            if numDoF < 2
                disp('Please specify kinematics with 2 or more DoF.');
                return;
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
            
            if numDoF >= 6
                diffDCM = Tn(1:3,1:3) * T0(1:3,1:3)';
                [rotAxis,rotAngle] = HebiUtils.rotMat2axAng(diffDCM);
            elseif numDoF >= 4
                rotAxis = cross(T0(1:3,3),Tn(1:3,3));
                
                if norm(rotAxis) > 1E-6
                    rotAxis = rotAxis / norm(rotAxis);
                    rotAngle = acos( dot(T0(1:3,3),Tn(1:3,3)) );
                    % Keep angle between +/- pi
                    if abs(rotAngle) > pi 
                        rotAngle = sign(rotAngle)*(abs(rotAngle) - 2*pi);
                    end
                else
                    rotAxis = [0 0 1];
                    rotAngle = 0;
                end
            end
                  
            % Get the joint angles for each waypoint
            IKAngles = nan(upSamplePoints, this.kin.getNumDoF());
            for i=1:upSamplePoints
                
                initAngles = upSampledAngles(i,:);

                if numDoF <= 3
                    % 3DoF IK
                    IKAngles(i,:) = getInverseKinematics( this.kin, ...
                        'xyz', upSampledXYZ(i,:)', ...
                        'initial', initAngles );
                elseif numDoF <= 5
                    % 5DoF IK
                    interpAngle = upSampledPhase(i)*rotAngle;
                    interpRotMat = axAng2rotMat(rotAxis,interpAngle);
                    cmdTipAxis = interpRotMat * T0(1:3,3);
                   
                    IKAngles(i,:) = getInverseKinematics( this.kin, ...
                        'xyz', upSampledXYZ(i,:)', ...
                        'tipAxis', cmdTipAxis, ...
                        'initial', initAngles );
                else
                    % 6DoF IK
                    interpAngle = upSampledPhase(i)*rotAngle;
                    interpRotMat = axAng2rotMat(rotAxis,interpAngle);
                    cmdDCM = interpRotMat*T0(1:3,1:3);
                    
                    IKAngles(i,:) = getInverseKinematics( this.kin, ...
                        'xyz', upSampledXYZ(i,:)', ...
                        'SO3', cmdDCM, ...
                        'initial', initAngles );  
                end
                
            end
            
            % Change Algorithm to MinJerkPhase if needed.  We still use the
            % old algorithm because it produces more smooth overall motion.
            % An update to the auto-timing heuristic is needed before
            % updating to UnconstrainedQP.  
            originalAlgorithm = this.getAlgorithm;
            if ~strcmp(originalAlgorithm,'MinJerkPhase')
                c = onCleanup(@()this.setAlgorithm(originalAlgorithm));
                this.setAlgorithm('MinJerkPhase');
            end
            
            % Construct joint move trajectory over interpolated angles
            trajectory = this.newJointMove(IKAngles, varargin{:});

        end
        
        function [] = moveJoint(this, group, positions, varargin)
            % MOVEJOINT creates a joint-space trajectorythat moves 
            % between the given waypoints, and executes the trajectory on a
            % group of actuators.  The trajectoty will start and stop at 
            % the first and last waypoints and will move thru any 
            % intermediate waypoints without stopping.
            %
            %   THIS FUNCTION IS DEPRECATED AND WILL BE REMOVED IN A FUTURE
            %   VERSION OF THE API.
            %
            %   This method is a 'blocking' call, which means that the
            %   function will run until the execution of the trajectory is
            %   completed.  If you need to access and react to feedback
            %   while a trajectory is being executed, use NEWJOINTMOVE to
            %   make the trajectory and then us TRAJECTORY.GETSTATE to 
            %   directly access commmands in a loop.
            %
            %   MOVEJOINT is a convenience wrapper that is equivalent
            %   to manually calling the following:
            %
            %      traj = trajGen.newJointMove(positions);
            %      trajGen.executeTrajectory(group, traj);
            %
            %   Arguments:
            %
            %       group     - HebiGroup containing the actuators 
            %       positions - [numWaypoints x numModules matrix] of 
            %                   positions.
            %
            %   Please check the documentation for newJointMove and
            %   executeTrajectory (direct links below) for information 
            %   on available parameters for these functions.
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
            %   See also HebiTrajectoryGenerator, HebiTrajectory, 
            %   newJointMove, executeTrajectory.
            traj = this.newJointMove(positions);
            this.executeTrajectory(group, traj, varargin{:});
        end
        
        function [] = moveLinear(this, group, positions, varargin)
            % moveLinear moves between waypoints in straight lines in
            % workspace, based on kinematics that were defined when setting
            % up the trajectory with HebiTrajectoryGenerator(kin).
            %
            %   THIS FUNCTION IS DEPRECATED AND WILL BE REMOVED IN A FUTURE
            %   VERSION OF THE API.
            %
            %   This method works the same as moveJoint() with the only
            %   difference being that the resulting trajectory represents
            %   an approximate straight line in cartesian workspace
            %   coordinates.
            %
            %   NOTE:
            %   Because the underlying IK relies on a local optimizer, the
            %   generated trajectory and waypoints for large motions, 
            %   particularly large changes in orientation, may be poor.
            %   Therefore it is recommended that you use this function for 
            %   moves that are in similar parts of the workspace and have 
            %   relatively small change in orientation of the end-effector.
            %
            %   This method is a 'blocking' call, which means that the
            %   function will run until the execution of the trajectory is
            %   completed.  If you need to access and react to feedback
            %   while a trajectory is being executed, use NEWJOINTMOVE to
            %   make the trajectory and then us TRAJECTORY.GETSTATE to 
            %   directly access commmands in a loop.
            %
            %   MOVELINEAR is a convenience wrapper that is equivalent
            %   to manually calling the following:
            %
            %      traj = trajGen.newLinearMove(positions);
            %      trajGen.executeTrajectory(group, traj);
            %
            %   Please refer to the documentation of newLinearMove and
            %   executeTrajectory for more information.
            %
            %   See also moveJoint, HebiTrajectoryGenerator, HebiTrajectory, 
            %   newLinearMove, newJointMove, executeTrajectory.
            traj = this.newLinearMove(positions);
            this.executeTrajectory(group, traj, varargin{:});
        end
        
        function [] = executeTrajectory(this, varargin)
            % executeTrajectory executes a trajectory and returns when done
            %
            %   Arguments:
            %
            %       group     - HebiGroup containing the actuators 
            %       positions - [numWaypoints x numModules matrix] of 
            %                   positions.
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
            %       % Create trajectory generator with defined kinematic
            %       % model for  acceleration compensation
            %       kin = HebiKinematics();
            %       kin.addBody('X5-4');
            %       kin.addBody('X5-Link', 'ext', 0.325, 'twist', 0);
            %       kin.addBody('X5-1');
            %       kin.addBody('X5-Link', 'ext', 0.325, 'twist', 0);
            %       kin.addBody('X5-1');
            %       trajGen = HebiTrajectoryGenerator(kin);
            %
            %       % Create trajectory w/ automatically determined time
            %       positions = [0 0 0; 1 1 1];
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
            %   See also HebiTrajectoryGenerator, newJointMove, newLinearMove
            
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
            
            % Sanity check
            useEfforts = p.EnableDynamicsComp || ~isempty(p.GravityVec);
            if useEfforts && isempty(this.kin)
                error(['To compensate for gravity or accelerations HebiTrajectoryGenerator ' ...
                    'needs to be initialized with a HebiKinematics object.']);
            end
            
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
                if t > duration
                    t = duration;
                end
                [pos, vel, accel] = traj.getState(t);
                
                % Convert accelerations to efforts
                effort = zeros(size(pos));
                if p.EnableDynamicsComp
                    accelCompEffort = this.kin.getDynamicCompEfforts(...
                        jacobianPosition, ...
                        pos, ...
                        vel, ...
                        accel);
                    effort = effort + accelCompEffort;
                end
                
                % Account for gravity if applicable
                if ~isempty(p.GravityVec)
                    gravCompEffort = this.kin.getGravCompEfforts(...
                        jacobianPosition, ...
                        p.GravityVec);
                    effort = effort + gravCompEffort;
                end
                
                % Offset for springs etc., if applicable
                if useEfforts && ~isempty(p.EffortOffset)
                    effort = effort + p.EffortOffset;
                end
                
                % Disable effort control if not needed
                if ~useEfforts
                    effort(:) = nan;
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
        
        function this = HebiTrajectoryGenerator(arg1)
            % Provides methods to generate trajectories
            
            % Backing Java object
            this.obj = javaObject(HebiTrajectoryGenerator.className);
            
            % Optional velocity limits / kinematics
            if nargin > 0 && ~isempty(arg1)
                
                if isa(arg1, 'HebiKinematics')
                    this.kin = arg1;
                    arg1 = this.kin.obj;
                end
                
                setVelocityLimits(this.obj, arg1);
                
            end
            
            % Initialize defaultable properties
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

