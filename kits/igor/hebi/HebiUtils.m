classdef (Sealed) HebiUtils
    % HebiUtils contains general utilitites for HEBI tools
    %
    %   HebiUtils Methods:
    %   now             - returns the current timestamp [s]
    %   convertGroupLog - converts a binary log into readable formats
    %   newGroupFromLog - replays data from a log file
    %   saveGains       - saves gains to disk (xml)
    %   loadGains       - loads gains from disk (xml)
    
    %   Copyright 2014-2016 HEBI Robotics, LLC.
    
    % Static API
    methods(Static)
        
        function out = now(varargin)
            % now returns the current timestamp in seconds
            %
            %   This method returns a timestamp that is consistent with the
            %   timestamps in group feedback. It can be used to approximate
            %   the delay between receiving data and getting it into MATLAB.
            %
            %   Example
            %      group = HebiLookup.newGroupFromFamily('*');
            %      fbk = group.getNextFeedbackFull();
            %      rtt = HebiUtils.now() - fbk.pcTxTime;
            out = javaMethod('now', HebiUtils.className,  varargin{:});
        end
        
        function out = convertGroupLog(varargin)
            % convertGroupLog converts a binary log file into various formats
            %
            %   This method is a more general version of HebiGroup.stopLog
            %   that can convert existing binary log files into readable
            %   formats. This is useful to recover data after a system
            %   crash, or to create alternative views of the data.
            %
            %   'InputFile' Argument
            %      Absolute or relative path to the binary log file.
            %
            %   'LogFormat' ('format') Parameter
            %      'Memory' converts to an in-memory struct. Large logs can
            %               result in out-of-memory errors. (default)
            %      'Csv'    converts data to a CSV file
            %      'Mat'    converts data to a MAT file
            %
            %   'View' Parameter
            %      'Simple' converts only simple feedback. This is appropriate
            %               for most users and results in much smaller log
            %               files. (default)
            %      'Full'   converts all available feedback. This is appropriate
            %               for advanced users that need additional timestamps
            %               or data from less common sensors.
            %
            %    Example
            %       % 1) Create a binary log
            %       group = HebiLookup.newGroupFromFamily('*');
            %       binaryFile = group.startLog();
            %       pause(5);
            %       tmp = group.stopLog();
            %
            %       % 2) Convert manually to .MAT file
            %       matFile = HebiUtils.convertGroupLog(binaryFile, 'format', 'mat');
            %       log = load(matFile);
            %       plot(log.time log.position);
            %
            %   See also HebiGroup.startLog, HebiGroup.stopLog
            out = javaMethod('convertGroupLog', HebiUtils.className,  varargin{:});
        end
        
        function group = newGroupFromLog(varargin)
            % newGroupFromLog replays data from a log file
            %
            %   This method creates a group that retrieves data from a log
            %   file rather than from the network. This can be very useful
            %   to test algorithms that need to run on live group data.
            %
            %   The replayed data is deterministic and is suitable for unit
            %   tests.
            %
            %   Example
            %      % Create a log
            %      group = HebiLookup.newGroupFromFamily('*');
            %      logFile = group.startLog();
            %      pause(10);
            %      logStruct = group.stopLog();
            %
            %      % Replay log until the end
            %      group = HebiUtils.newGroupFromLog(logFile);
            %      fbk = group.getNextFeedback();
            %      while ~isempty(group.getNextFeedback(fbk))
            %          display(fbk.position);
            %      end
            %
            %   See also HebiUtils, HebiLookup, HebiGroup
            group = HebiGroup(javaMethod('newGroupFromLog', HebiUtils.className,  varargin{:}));
        end
        
        function out = saveGains(varargin)
            % saveGains saves gains to disk (xml)
            %
            %   This method saves gains from a GainStruct in a human 
            %   readable format on disk. Note that this includes only the
            %   control strategy and the corresponding gains, and may
            %   exclude some fields such as 'time' or 'mStopStrategy'.
            %
            %   Example
            %      % Save gains to xml
            %      gains = GainStruct(); % alt: group.getGains();
            %      gains.positionKp = [1 1 1 1];
            %      gainFile = HebiUtils.saveGains(gains, 'MyGains');
            %      display(gainFile);
            %
            %   See also HebiUtils, loadGains
            out = javaMethod('saveGains', HebiUtils.className,  varargin{:});
        end
          
        function out = loadGains(varargin)
            % loadGains loads gains from disk (xml)
            %
            %   This method loads gains from a human readable file into a
            %   GainStruct. Note that this includes only the control
            %   strategy and the corresponding gains, and may exclude some
            %   fields such as 'time' (set to now) or 'mStopStrategy'.
            %
            %   Example
            %      % create dummy file
            %      gainFile = HebiUtils.saveGains(GainStruct(), 'MyGains');  
            %
            %      % load gains from xml w/ dummy data
            %      gains = HebiUtils.loadGains(gainFile);
            %
            %   See also HebiUtils, saveGains
            out = javaMethod('loadGains', HebiUtils.className,  varargin{:});
        end
        
    end
    
    properties(Constant, Access = private, Hidden = true)
        className = hebi_load('HebiUtils');
    end
    
    % Non-API Methods for MATLAB compliance
    methods(Access = public, Hidden = true)
        
        function this = HebiUtils
            % HEBI Robotics Utilities
        end
        
        function disp(this)
            disp(javaObject(HebiUtils.className));
        end
        
    end
    
    % Non-API Static methods for MATLAB compliance
    methods(Access = public, Static, Hidden = true)
        
        function varargout = methods(varargin)
            instance = javaObject(HebiUtils.className);
            switch nargout
                case 0
                    methods(instance, varargin{:});
                    varargout = {};
                otherwise
                    varargout{:} = methods(instance, varargin{:});
            end
        end
        
        function varargout = fields(varargin)
            instance = javaObject(HebiUtils.className);
            switch nargout
                case 0
                    fields(instance, varargin{:});
                    varargout = {};
                otherwise
                    varargout{:} = fields(instance, varargin{:});
            end
        end
        
    end
    
end