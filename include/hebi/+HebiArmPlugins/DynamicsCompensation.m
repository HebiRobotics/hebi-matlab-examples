classdef DynamicsCompensation < HebiArmPlugin
    % DynamicsCompensation arm plugin
    %
    % Compensates for the mass moved by joint accelerations.
    
    methods
        
        function this = DynamicsCompensation()
        end
        
        function [] = update(this, arm)

            % Only apply if there is a trajectory
            if isempty(arm.state.cmdPos)
                return;
            end

            % Add dynamics comp efforts
            dynamicsCompEfforts = arm.kin.getDynamicCompEfforts(...
                    arm.state.fbk.position, ...
                    arm.state.cmdPos, ...
                    arm.state.cmdVel, ...
                    arm.state.cmdAccel);

            % Add to efforts
            rampScale = this.getRampScale(arm.state.time);
            arm.state.cmdEffort = arm.state.cmdEffort + dynamicsCompEfforts .* rampScale;
            
        end
        
    end
    
end
