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
        Ki double = zeros(6,1); % (N/m*sec) or (Nm/rad*sec)
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
            
            if arm.state.dt == 0
                this.iError(:,:) = 0; % Reset on restarts
            else
                this.iError = this.iError + posError * arm.state.dt;
            end
            
            % Calculate Impedance Control Wrenches and Appropriate Joint
            % Torques
            pWrench = zeros(6,1);
            iWrench = zeros(6,1);
            dWrench = zeros(6,1);

            if this.gainsInEndEffectorFrame
                armTipRotFbk = actualTipFK(1:3,1:3);
                posError(1:3) = armTipRotFbk' * posError(1:3); % linear error
                posError(4:6) = armTipRotFbk' * posError(4:6); % rotational error     
                velError(1:3) = armTipRotFbk' * velError(1:3); % linear velocity error
                velError(4:6) = armTipRotFbk' * velError(4:6); % rotational velocity error
            end

            this.iError = this.iError + posError * dt;

            pWrench(1:3) = this.Kp(1:3) .* posError(1:3); % linear force
            pWrench(4:6) = this.Kp(4:6) .* posError(4:6); % rotational torque
            iWrench(1:3) = this.Ki(1:3) .* this.iError(1:3); % linear force
            iWrench(4:6) = this.Ki(4:6) .* this.iError(4:6); % rotational torque    
            dWrench(1:3) = this.Kd(1:3) .* velError(1:3); % linear damping
            dWrench(4:6) = this.Kd(4:6) .* velError(4:6); % rotational damping

            if this.gainsInEndEffectorFrame
                pWrench(1:3) = armTipRotFbk * pWrench(1:3);
                pWrench(4:6) = armTipRotFbk * pWrench(4:6);
                iWrench(1:3) = armTipRotFbk * iWrench(1:3);
                iWrench(4:6) = armTipRotFbk * iWrench(4:6);
                dWrench(1:3) = armTipRotFbk * dWrench(1:3);
                dWrench(4:6) = armTipRotFbk * dWrench(4:6);
            end
            
            % Add impedance efforts to effort output
            impedanceEfforts = J_armTip' * (pWrench + + iWrench + dWrench);
            arm.state.cmdEffort = arm.state.cmdEffort + impedanceEfforts';
            
        end
        
    end
    
end