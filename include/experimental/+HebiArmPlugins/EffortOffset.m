classdef EffortOffset < HebiArmPlugin
    % EffortOffsetPlugin adds constant offsets to the 

    properties
       effortOffset double;
    end
    
    methods
        
        function this = EffortOffset(effortOffset)
            this.effortOffset = effortOffset;
        end
        
        function [] = update(this, arm)
            arm.state.cmdEffort = arm.state.cmdEffort + this.effortOffset;
        end
        
    end

end

