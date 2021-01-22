classdef ImpedanceController < HebiArmPlugin
    % ImpedanceControlPlugin
    %
    % % Impendance Control Gains
    % NOTE: The gains corespond to:
    % [ trans_x trans_y trans_z rot_x rot_y rot_z ]
    %
    % Translations and Rotations can be specified in the
    % base frame or in the end effector frame.
    
    properties
        gainsInEndEffectorFrame logical = true;
        
        Kp double = zeros(6,1); % (N/m) or (Nm/rad)
        Ki double = zeros(6,1); % (N/(m*sec)) or (Nm/(rad*sec))
        Kd double = zeros(6,1); % (N/(m/sec)) or (Nm/(rad/sec))
        
    end
    
    properties(Access = private)
        iError double = zeros(6,1);
    end
    
    methods
        
        function this = ImpedanceController()
        end
        
        function [] = update(this, arm)
            if isempty(arm.state.cmdPos) || isempty(arm.state.cmdVel)
                this.iError(:,:) = 0;
                return;
            end

            % Desired/Actual joint state
            cmdPos = arm.state.cmdPos;
            cmdVel = arm.state.cmdVel;
            fbkPos = arm.state.fbk.position;
            fbkVel = arm.state.fbk.velocity;
            
            % Get Updated Forward Kinematics and Jacobians
            desiredTipFK =  arm.kin.getForwardKinematicsEndEffector(cmdPos);
            actualTipFK = arm.kin.getForwardKinematicsEndEffector(fbkPos);
            J_armTip = arm.kin.getJacobianEndEffector(fbkPos);
            if this.gainsInEndEffectorFrame
                frameRot = actualTipFK(1:3,1:3); % end effector
            else
                frameRot = eye(3); % world frame
            end
            
            % Linear error is easy
            xyzError = desiredTipFK(1:3,4) - actualTipFK(1:3,4);
            
            % Rotational error involves calculating axis-angle from the
            % resulting error in S03 and providing a torque around that axis.
            errorRotMat = desiredTipFK(1:3,1:3) * actualTipFK(1:3,1:3)';
            [axis, angle] = HebiUtils.rotMat2axAng( errorRotMat );
            rotErrorVec = angle * axis;
            
            posError = [xyzError; rotErrorVec];
            velError = J_armTip * (cmdVel - fbkVel)';

            % Calculate Impedance Control Wrenches and Appropriate Joint
            % Torques
            posError = this.rotate(frameRot', posError);
            velError = this.rotate(frameRot', velError);
            this.iError = this.iError + posError * arm.state.dt;
            wrench = ...
                + this.Kp .* posError ...
                + this.Ki .* this.iError ...
                + this.Kd .* velError;
            wrench = this.rotate(frameRot, wrench);

            % Add impedance efforts to effort output
            impedanceEfforts = J_armTip' * wrench;
            arm.state.cmdEffort = arm.state.cmdEffort + impedanceEfforts';
            
        end
        
    end
    
    methods(Static, Hidden)
        function result = rotate(R, vec)
            result = zeros(6,1);
            result(1:3) = R(1:3,1:3) * vec(1:3); % linear component
            result(4:6) = R(1:3,1:3) * vec(4:6); % rotational component
        end
    end
    
end
