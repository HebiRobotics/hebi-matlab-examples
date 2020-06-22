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
        damperGains double = zeros(6,1); % (N/(m/sec)) or (Nm/(rad/sec))
        springGains double = zeros(6,1); % (N/m) or (Nm/rad)
    end
    
    properties(Access = private)
    end
    
    methods
        
        function this = ImpedanceController()
        end
        
        function [] = update(this, arm)
            if isempty(arm.state.cmdPos) || isempty(arm.state.cmdVel)
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
            
            % Linear error is easy
            xyzError = desiredTipFK(1:3,4) - actualTipFK(1:3,4);
            
            % Rotational error involves calculating axis-angle from the
            % resulting error in S03 and providing a torque around that axis.
            errorRotMat = desiredTipFK(1:3,1:3) * actualTipFK(1:3,1:3)';
            [axis, angle] = HebiUtils.rotMat2axAng( errorRotMat );
            rotErrorVec = angle * axis;
            
            if this.gainsInEndEffectorFrame
                xyzError = actualTipFK(1:3,1:3)' * xyzError;
                rotErrorVec = actualTipFK(1:3,1:3)' * rotErrorVec;
            end
            
            posError = [xyzError; rotErrorVec];
            velError = J_armTip * (cmdVel - fbkVel)';
            
             % Calculate Impedance Control Wrenches and Appropriate Joint
             % Torques
            springWrench = zeros(6,1);
            damperWrench = zeros(6,1);
            
            springWrench(1:3) = this.springGains(1:3) .* posError(1:3); 
            springWrench(4:6) = this.springGains(4:6) .* posError(4:6);
            
            if this.gainsInEndEffectorFrame
                springWrench(1:3) = actualTipFK(1:3,1:3) * springWrench(1:3); % linear force
                springWrench(4:6) = actualTipFK(1:3,1:3) * springWrench(4:6); % rotational torque 
            end
            
            damperWrench(1:3) = this.damperGains(1:3) .* velError(1:3); % linear damping
            damperWrench(4:6) = this.damperGains(4:6) .* velError(4:6); % rotational damping
            
            % Add impedance efforts to effort output
            impedanceEfforts = J_armTip' * (springWrench + damperWrench);
            arm.state.cmdEffort = arm.state.cmdEffort + impedanceEfforts';
            
        end
        
    end
    
end

