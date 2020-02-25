classdef EffortOffsetPlugin < handle
    % EffortOffsetPlugin adds constant offsets to the 

    properties
       effortOffset double;
    end
    
    methods
        
        function this = EffortOffsetPlugin(effortOffset)
            this.effortOffset = effortOffset;
        end
        
        function newState = update(this, newState, ~)
            newState.cmdEffort = newState.cmdEffort + this.effortOffset;
        end
        
    end

end

