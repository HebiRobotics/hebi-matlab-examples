classdef (Sealed) HebiTrajectory
    % HebiTrajectory represents a precomputed trajectory path
    %
    %   Trajectories are created by a HebiTrajectoryGenerator and provide
    %   a way to asynchronously move through a trajectory and to
    %   visualize the planned path.
    %
    %   HebiTrajectory Methods:
    %      getDuration     - returns the total duration [s]
    %      getState        - returns position/velocity/acceleration 
    %                        at any given time.
    %      getWaypointTime - returns the timestamp for each waypoint
    %
    %   Example
    %      % Create trajectory
    %      trajGen = HebiTrajectoryGenerator(kin);
    %      trajectory = trajGen.newJointMove([start; finish]);
    %
    %      % Visualize trajectory
    %      t = 0:0.01:trajectory.getDuration();
    %      [pos, vel, accel] = trajectory.getState(t);
    %      figure(); hold on;
    %      plot(t, pos);
    %      plot(t, vel);
    %      plot(t, accel);
    %
    %      % Manually execute position/velocity/effort trajectory
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
    %               fbk.position, ... % Used for calculating jacobian
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
    %   See also HebiTrajectoryGenerator,
    %   HebiTrajectoryGenerator.newJointMove,
    %   HebiTrajectoryGenerator.executeTrajectory

    %   Copyright 2014-2017 HEBI Robotics, Inc.
        
    %% API Methods
    methods(Access = public)
        
        function duration = getDuration(this)
            % getDuration returns the total duration of the trajectory [s]
            %
            %   This is equivalent to 
            %     
            %       time = trajectory.getWaypointTime();
            %       duration = time(end) - time(1);
            %
            %   See also getWaypointTime
            duration = this.duration;
        end
        
          function out = getWaypointTime(this)
            % getWaypointTime returns the input time for each waypoint [s]
            %
            %   This method returns the time for each waypoint that served
            %   as input to calculate this trajectory. If the time vector
            %   has been specified manually, the result is equivalent to
            %
            %       offset = time(1);
            %       result = ((time - offset) * speedFactor) + offset
            %
            %   This is useful for, e.g., plotting the waypoints on top of
            %   a whole trajectory.
            %
            %   Example
            %       % Plot waypoints on top of trajectory
            %       % (create trajectory via trajectory generator)
            %       time = trajectory.getWaypointTime();
            %       positionWaypoints = trajectory.getState(time);
            %       
            %       % Plot continuous trajectory
            %       t = time(1):0.01:time(end); % 100 Hz steps
            %       position = trajectory.getState(t);
            %       plot(t, position);
            %
            %       % Add waypoints as discrete points
            %       hold on; 
            %       plot(time, positionWaypoints, 'o');
            %
            %   See also getDuration, HebiTrajectoryGenerator.newJointMove
            if isempty(this.time)
               this.time = getWaypointTime(this.obj); 
            end
            out = this.time;
        end
        
        function [position, velocity, acceleration] = getState(this, varargin)
            % getState returns the state of [pos, vel, accel] at time t
            %
            %   Arguments:
            %       time - scalar or vector of times
            %
            %   Note that the trajectory state is only defined for 
            %   times between zero and the total duration.
            %
            %   Example
            %      % Plot position over entire trajectory
            %      t = 0:0.01:trajectory.getDuration();
            %      [pos, vel, accel] = trajectory.getState(t);
            %      plot(t, pos);
            %
            %   See also HebiTrajectory, HebiTrajectoryGenerator
            state = struct(getState(this.obj, varargin{:}));
            position = state.position;
            velocity = state.velocity;
            acceleration = state.acceleration;
        end
        
    end
    
    %% Hidden Properties    
    properties(Access = private, Hidden = true)
        duration; % cached duration
        time = []; % cached time vector
        obj;
    end
    
    properties(Constant, Access = private, Hidden = true)
        className = hebi_load('HebiTrajectory');
    end
    
    %% Hidden Methods
    methods(Access = public, Hidden = true)
        function this = HebiTrajectory(obj)
            % HebiTrajectory represents a precomputed trajectory path
            switch(nargin)
                case 0
                    % default constructor
                    this.obj = javaObject(HebiTrajectory.className);
                case 1
                    % wrapper
                    if(~isa(obj, HebiTrajectory.className))
                        error('invalid argument');
                    end
                    this.obj = obj;
                    this.duration = getDuration(obj);
            end
        end
        
         function disp(this)
            disp(this.obj);
        end
    end
    
    
end

