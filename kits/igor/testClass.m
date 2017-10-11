classdef  testClass < handle

    
    properties(Access = public)
        
       % State Variables
       test;
    end

    
    methods
        
        % Make the initial parameters
        function this = testClass()

            % Initialize state
            this.test = 0;
            
        end
        
        function this = update(this, newValue, newValue2)
            
            oldValue = this.test
            this.test = newValue2;
        end
    end
    

end

