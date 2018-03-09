classdef (Sealed) DemoUtils
    % DemoUtils contains useful functions that are often used in demos
    %
    %   DemoUtils Methods:
    %
    %   retryOnError - continuously executes a function until it encounters
    %                  no exceptions
    
    %   Copyright 2014-2018 HEBI Robotics, Inc.
    
    % Static API
    methods(Static)
        
        function varargout = retryOnError(func, onErrorFunc)
            % retryOnError continuously executes a function until it
            % encounters no exceptions.
            %
            %    func = @()HebiLookup.newGroupFromName('Family','Name');
            %    group = DemoUtils.retryOnError(func);
            %
            %   This method is useful, e.g., when modules should be
            %   searched continuously and throwing an error or continuing
            %   the script would be undesired.
            %
            %   The 2nd parameter is a function handle that gets called
            %   whenever an error/exception occurs. Defaults to no action.
            %   retries.
            if nargin < 2
               onErrorFunc = [];
            end
            while true
                try
                    % Execute
                    if(nargout == 0)
                        func();
                    else
                        varargout{:} = func();
                    end
                    break;
                catch
                    % Ignore exceptions
                    if ~isempty(onErrorFunc)
                        onErrorFunc();
                    end
                    pause(0.1);
                end
            end
        end
        
        function [] = bootIntoApplication(group, pauseTime)
            % bootIntoApplication makes sure all modules are in
            % application mode
            if nargin < 2
                pauseTime = 0.5;
            end
            
            %loop until all modules are in application
            while true
                moduleInfo = group.getInfo();
                inBootloader = strcmp('bootloader', moduleInfo.firmwareMode);
                
                %if any modules are in bootloader just try to boot all of them
                if (any(inBootloader))
                    robotGroup.send('boot',true);
                else
                    break;
                end
                
                pause(pauseTime);
            end
            
        end
        
        function [] = sendGains(group, gains)
           %  sendGains is a method that makes it very likely that a set of
           %  gains was actually received by the target group.
           
           % Disable feedback frequency to reduce messages
           freq = group.getFeedbackFrequency();
           cleaner = onCleanup(@()group.setFeedbackFrequency(freq));
           group.setFeedbackFrequency(0);
           pause(0.5);
           
           % Send multiple times
           for i = 1:5
               group.send('gains', gains);
               pause(0.1);
           end
            
        end
        
    end
    
    % Utility methods for use within other utility methods
    methods(Access = public, Static, Hidden = true)
        
    end
    
end
