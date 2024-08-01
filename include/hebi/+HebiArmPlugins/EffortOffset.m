classdef EffortOffset < HebiArmPlugin
    % EffortOffsetPlugin adds constant offsets to the 

    properties
       offset double;
    end
    
    methods
        
        function this = EffortOffset(effortOffset)
            this.offset = effortOffset;
        end
        
        function [] = update(this, arm)
            rampScale = this.getRampScale(arm.state.time);
            arm.state.cmdEffort = arm.state.cmdEffort + this.offset .* rampScale;
        end
        
    end

end

