classdef HebiArm
    %HEBIARM Summary of this class goes here
    %   Detailed explanation goes here
    %
    % Expected Usage:
    %
    % kin = HebiKinematics('myArm.hrdf');
    % arm = HebiArm(kin);
    % arm.setSpringGains([500; 500; 0; 5; 5; 5]);  % (N/m) or (Nm/rad)
    % arm.setDamperGains([10; 10; 0; .1; .1; .1;]); % (N/(m/sec)) or (Nm/(rad/sec))
    % % ... other setup ...
    %
    % group = HebiLookup.newGroupFromNames(...)
    % cmd = CommandStruct();
    % while true
    %
    %    % Execute trajectory to specified wapoint
    %    fbk = group.getNextFeedbackFull();
    %    isFinished = arm.update(fbk, cmd);
    %    group.send(cmd); % User still has access to change pos/vel/effort
    %
    %    % Replan immediately or add next waypoint once arm has arrived
    %    if(isFinished)
    %       % Go to next waypoint
    %       target = kin.getIK(... whatever desired target ...)
    %       arm.moveLinearTo(target, time);
    %    end
    %
    % end 
    %
    %
    % TODO: what would be a good way to switch to grav-comp zero-torque
    % mode?
    
    properties
        kin HebiKinematics;
        trajStartTime double;
        traj HebiTrajectory;
        prevState struct;
        enableGravComp = true;
        enableDynamicsComp = true;
        % TODO: speed factor?
        % TODO: initialization to first starting waypoint?
        % TODO: add a mask to allow selecting a subset of feedback (e.g. hexapod)
    end
    
    methods
        
        function [] = moveLinearTo(this, position, duration)
            %MOVELINEARTO Summary
            %   Moves from the previously commanded state to the desired
            %   target state in a straight line in carthesian space. End
            %   conditions of zero accel and vel.
            %
            % The goal is specified in joint angles to avoid issues with
            % degrees of freedom that are not within the workspace.
            % Interpolating between two valid joint positions should always
            % result in something mathematically possible (right?). No
            % collision checks.
            
            % TODO: calculate carthesian coordinates of previous state,
            % then linearly interpolate, and back out joint angles.
            
            % TODO: then call moveJointTo with the specified waypoints
            
        end
        
        function [] = moveJointTo(this, position, duration)
            %MOVEJOINTTO Summary
            %   Moves from the previously commanded state to the desired
            %   target state in joint-space. End conditions of zero accel
            %   and vel.
            
            % TODO: replan trajectory from previous state to desired joint
            % angles within given duration (or auto-duration if not
            % specified)
            
            % TODO: support multi-waypoints
            this.trajStartTime = this.prevState.time;

        end
        
        function isFinished = update(this, fbk, cmd)
            
            % Evaluate trajectory state
            t = fbk.time - this.trajStartTime;
            [cmdPos, cmdVel, cmdAccel] = this.traj.getState(t);
            this.prevState.time = fbk.time;
            this.prevState.pos = cmdPos;
            this.prevState.vel = cmdVel;
            this.prevState.accel = cmdAccel;
            % TODO: make sure state is not extrapolated, i.e., max(t,
            % duration)
            
            % Initialize efforts
            numDoF = this.kin.getNumDoF();
            cmdEffort = zeros(1, numDoF);
            
            % Compensate for accelerations due to gravity
            if(this.enableGravComp)
                cmdEffort = cmdEffort + this.getGravCompEfforts(fbk);
            end
            
            % Compensate for joint accelerations
            if(this.enableDynamicsComp)
                cmdEffort = cmdEffort + this.kin.getDynamicCompEfforts(...
                    fbk.position, cmdPos, cmdVel, cmdAccel);
            end
            
            % TODO: add effort offsets for e.g. spring. Maybe this should
            % live in user code?
            
            % TODO: add efforts to add an end effector wrench (local or
            % world coordinates?)
            
            % TODO: add impedance controller
            
            % TODO: add enable/disable for pos/vel? e.g. what happens for 
            % pure impedance controller? Maybe remove pos/vel in user code?
                        
            cmd.position = cmdPos;
            cmd.velocity = cmdVel;
            cmd.effort = cmdEffort; % TODO: remove if zeros? Strat 3 issues?
            
            isFinished = (t >= this.traj.getDuration());
            
            % TODO: Maybe in addition to the boolean value, maybe return a
            % relative time? e.g. trajectory 50% completed. This may be
            % useful for doing things with the gripper or additional
            % torques (e.g. wiggle-fit).
            
        end
        
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
end

