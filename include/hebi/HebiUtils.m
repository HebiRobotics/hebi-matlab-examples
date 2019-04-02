classdef (Sealed) HebiUtils
    % HebiUtils contains general utilitites for HEBI tools
    %
    %   HebiUtils Methods:
    %
    %   now                - returns the current timestamp [s]
    %
    %   saveGains          - saves group gains to disk (XML)
    %   loadGains          - loads group gains from disk (XML)
    %
    %   newGroupFromLog    - generates a group from a .hebilog file that
    %                        you can use play back data using getNextFeedback.
    %
    %   newImitationGroup  - creates an imitation group for testing
    %
    %   loadGroupLog       - loads a binary .hebilog file into memory
    %   loadGroupLogsUI    - shows a UI dialog to load one or more logs.
    %
    %   readGroupLogInfo   - reads the first info and gains struct from
    %                        a .hebilog file
    %
    %   convertGroupLog    - converts a binary .hebilog file into a readable
    %                        format, either in memory or files like CSV or MAT.
    %   convertGroupLogsUI - shows a UI dialog to chose one or more log
    %                        files to convert.
    %
    %   plotLogs           - visualizes feedback data from one or more log
    %                        files in a formatted and labeled plot.
    %
    %   plotTrajectory     - visualizes the positions, velocities, and
    %                        accelerations of a trajectory produced by
    %                        HebiTrajectoryGenerator.
    %
    %   quat2rotMat        - converts orientations represented by a unit
    %                        quaternion to a rotation matrix.

    %   Copyright 2014-2018 HEBI Robotics, Inc.
    
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
            %      roundTripTime = HebiUtils.now() - fbk.pcTxTime;
            out = javaMethod('now', HebiUtils.className,  varargin{:});
        end
        
        function varargout = loadGroupLog(varargin)
            % loadGroupLog loads a binary .hebilog file into memory
            %
            %   This method converts binary log files into a log struct that 
            %   contains all the feedback from a group over time that the group
            %   was being logged.  Format of the log struct mirrors that of the
            %   feedback struct, except that there are multiple rows of
            %   data, each corresponding to a successive point in time.
            %
            %   'InputFile' Argument (required)
            %      Absolute or relative path to the binary log file.
            %
            %   'View' Parameter (optional)
            %      'Simple' converts only simple feedback. This is appropriate
            %               for most users and results in much smaller log
            %               files. (default)
            %      'Full'   converts all available feedback. This is appropriate
            %               for advanced users that need additional timestamps
            %               or data from less common sensors.
            %
            %   Additionally, this method returns info and gains if the
            %   corresponding data is contained within the log. For
            %   logs recorded before version 1.1, very short
            %   logs, and logs that were taken while the lookup was
            %   disabled, info and gains may return empty.
            %
            %    Example:
            %       % 1) Create a binary .hebilog file
            %       group = HebiLookup.newGroupFromFamily('*');
            %       logFile = group.startLog();
            %       pause(5);
            %       group.stopLog('format','raw'); % Does not convert log to
            %                                      % memory.
            %
            %       % 2) Load the log from the binary file
            %       hebiLog = HebiUtils.loadGroupLog(logFile);
            %       plot(hebiLog.time, hebiLog.position);
            %
            %       % 3) Also read info and gains
            %       [hebiLog, info, gains] = HebiUtils.loadGroupLog(logFile)
            %
            %   See also HebiGroup.startLog, HebiGroup.stopLog,
            %   loadGroupLogsUI, convertGroupLog
            [varargout{1:nargout}] = HebiUtils.convertGroupLog(varargin{:}, 'LogFormat', 'memory');
        end
        
        function varargout = convertGroupLog(varargin)
            % CONVERTGROUPLOG converts a binary log file into various formats
            %
            %   This method is a more general version of HebiGroup.stopLog
            %   that can convert existing binary log files into other readable
            %   formats. This is useful to recover data after a system
            %   crash, or to create alternative views of the data.
            %
            %   'InputFile' Argument (required)
            %      Absolute or relative path to the binary log file.
            %
            %   'LogFormat' ('format') Parameter (optional)
            %      'Memory' converts to an in-memory struct. Large logs can
            %               result in out-of-memory errors. (default)
            %      'Csv'    converts data to a CSV file
            %      'Mat'    converts data to a MAT file
            %      'Raw'    returns the input path
            %
            %   'View' Parameter  (optional)
            %      'Simple' converts only simple feedback. This is appropriate
            %               for most users and results in much smaller log
            %               files. (default)
            %      'Full'   converts all available feedback. This is appropriate
            %               for advanced users that need additional timestamps
            %               or data from less common sensors.
            %
            %   Additionally, this method returns info and gains if the
            %   corresponding data is contained within the log. For
            %   logs recorded before version 1.1, very short
            %   logs, and logs that were taken while the lookup was
            %   disabled, info and gains may return empty.
            %
            %    Example:
            %       % 1) Create a binary .hebilog file
            %       group = HebiLookup.newGroupFromFamily('*');
            %       logFile = group.startLog();
            %       pause(5);
            %       group.stopLog('format','raw'); % Does not convert log to
            %                                      % memory.
            %
            %       % 2) Convert manually to .MAT file
            %       matFile = HebiUtils.convertGroupLog(logFile, 'format', 'mat');
            %       hebiLog = load(matFile);
            %       plot(hebiLog.time, hebiLog.position);
            %
            %       % 3) Also read info and gains
            %       [hebiLog, info, gains] = HebiUtils.convertGroupLog(logFile)
            %
            %   See also HebiGroup.startLog, HebiGroup.stopLog,
            %   convertGroupLogsUI, loadGroupLog.
            varargout = {javaMethod('convertGroupLog', HebiUtils.className,  varargin{:})};
            if nargout > 1
                inputFile = varargin{1};
                [varargout{2}, varargout{3}] = HebiUtils.readGroupLogInfo(inputFile);
            end
        end
        
        function group = newGroupFromLog(varargin)
            % NEWGROUPFROMLOG replays data from a log file
            %
            %   This method creates a group that retrieves data from a log
            %   file rather than from the network. This can be very useful
            %   to test algorithms that need to run on live group data.
            %
            %   The replayed data is deterministic and is suitable for unit
            %   tests.
            %
            %   Example:
            %      % Create a log
            %      group = HebiLookup.newGroupFromFamily('*');
            %      logFile = group.startLog();
            %      pause(10);
            %      logStruct = group.stopLog();
            %
            %      % Replay log until the end
            %      group = HebiUtils.newGroupFromLog(logFile);
            %      while true
            %          fbk = group.getNextFeedback();
            %          if isempty(fbk)
            %              % stop at the end of the log
            %              break;
            %          end
            %          display(fbk.position);
            %      end
            %
            %   See also HebiUtils, HebiLookup, HebiGroup
            group = HebiGroup(javaMethod('newGroupFromLog', HebiUtils.className,  varargin{:}));
        end
        
        function group = newImitationGroup(varargin)
            % NEWIMITATIONGROUP creates an imitation group for testing
            %
            %   An imitation group behaves mostl the same way as a group
            %   created by HebiLookup, without the requirement to have
            %   physically connected devices on the network.
            %
            %   The main differences are as follows
            %
            %   * Commands immediately set the corresponding feedback without
            %     veryfing physical feasibility. Values will be visible in
            %     the next feedback.
            %
            %   * Imitation devices do not update setting fields (e.g. name)
            %
            %   * There is no feedback for physical sensors such as IMU data
            %
            %   Example:
            %     % Create an imitation group
            %     numModules = 1;
            %     group = HebiUtils.newImitationGroup(numModules);
            % 
            %     % Generate log data
            %     cmd = CommandStruct();
            %     logFile = group.startLog();
            %     t0 = group.getNextFeedback().time;
            %     t = 0;
            %     while t < 5
            %         fbk = group.getNextFeedback();
            %         t = fbk.time - t0;
            %         cmd.position = sin(2*pi*t);
            %         group.send(cmd);
            %     end
            %     logStruct = group.stopLog();
            % 
            %     % Visualize commands
            %     plot(logStruct.time, logStruct.position);
            %     hold on;
            %     plot(logStruct.time, logStruct.positionCmd, ':');
            %
            %   See also HebiUtils, HebiLookup, HebiGroup
            group = HebiGroup(javaMethod('newImitationGroup', HebiUtils.className,  varargin{:}));
        end
        
        function out = saveGains(varargin)
            % SAVEGAINS saves gains for group to disk (xml)
            %
            %   This method saves gains from a GainStruct in a human
            %   readable format on disk. Note that this includes only the
            %   control strategy and the corresponding gains, and may
            %   exclude some fields such as 'time' or 'mStopStrategy'.
            %
            %   Examples:
            %      % Make a new set of gains and save to XML
            %      gains = GainStruct(); % alt: group.getGains();
            %      gains.positionKp = [1 1 1 1];
            %      gainFile = HebiUtils.saveGains(gains, 'MyGains');
            %      display(gainFile);
            %
            %      % Save gains that are currently set for a group
            %      gains = group.getGains();
            %      gainFile = HebiUtils.saveGains(gains, 'MyGains');
            %      display(gainFile);
            %
            %   See also HebiUtils, loadGains, GainStruct.
            out = javaMethod('saveGains', HebiUtils.className,  varargin{:});
        end
        
        function out = loadGains(varargin)
            % LOADGAINS loads gains for group from disk (xml)
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
            %   See also HebiUtils, saveGains, GainStruct.
            out = javaMethod('loadGains', HebiUtils.className,  varargin{:});
        end
        
        function [info, gains, safetyParams] = readGroupLogInfo(hebiLogFile)
            % readGroupLogInfo reads the first info and gains struct
            % from a binary .hebilog file
            %
            %   This method will return empty if the log does not contain
            %   any info or log data.
            %
            %   Example:
            %       [info, gains, safetyParams] = HebiUtils.readGroupLogInfo(path);
            %
            %   Example:
            %       % Get info for all logs selected via a GUI
            %       fileNames = HebiUtils.convertGroupLogsUI('format','raw');
            %       [infos, gains, safetyParams] = cellfun(@HebiUtils.readGroupLogInfo, ...
            %           fileNames, 'UniformOutput', false);
            
            % Read log file
            group = HebiUtils.newGroupFromLog(hebiLogFile);
            
            % Step through log until there is enough data to populate info,
            % or we reach the end of the file
            fbk = group.getNextFeedback();
            while isempty(group.getInfo()) && ~isempty(fbk)
                fbk = group.getNextFeedback(fbk);
            end
            
            % Read info/gains
            info  = group.getInfo();
            gains = group.getGains();
            safetyParams = group.getSafetyParams();
            
        end
        
        function [ varargout ] = loadGroupLogsUI( varargin )
            %LOADGROUPLOGSUI shows a dialog to choose one or more .hebilog files 
            %to load into memory.
            %
            %   This method lets users choose one or more raw .hebilog
            %   files via a file selector dialog and loads them into memory. 
            %   The output is a cell array of the conversion result for each 
            %   selected file.
            %
            %   The optional parameters are the same as LOADGROUPLOG.
            %
            %   'View' Parameter (optional)
            %      'Simple' converts only simple feedback. This is appropriate
            %               for most users and results in much smaller log
            %               files. (default)
            %      'Full'   converts all available feedback. This is appropriate
            %               for advanced users that need additional timestamps
            %               or data from less common sensors.
            %
            %    Example:
            %       % Load selected files and plot position feedback
            %       hebiLogs = HebiUtils.loadGroupLogsUI();
            %       HebiUtils.plotLogs(hebiLogs, 'position');
            %
            %    Example:
            %       % Load selected files as 'full' structs
            %       hebiLogs = HebiUtils.loadGroupLogsUI('View', 'full');
            %
            %    Example:
            %       % Add info/gains if available
            %       [hebiLogs, infos, gains] = HebiUtils.loadGroupLogsUI();
            %
            % See also HebiGroup.stopLog, convertGroupLog, loadGroupLog.
                        
            [varargout{1:nargout}] = HebiUtils.convertGroupLogsUI(varargin{:}, 'LogFormat', 'memory');
        end
        
        function [ varargout ] = convertGroupLogsUI( varargin )
            %CONVERTGROUPLOGSUI shows a dialog to choose one or more
            %.hebilog files to convert
            %
            %   This method lets users choose one or more raw .hebilog
            %   files via a file selector dialog and converts them into a 
            %   MATLAB readable format. The output is a cell array of the
            %   conversion result for each selected file.
            %
            %   The optional parameters are the same as CONVERTGROUPLOG.
            %
            %   'LogFormat' ('format') Parameter
            %      'Memory' converts to an in-memory struct. Large logs can
            %               result in out-of-memory errors. (default)
            %      'Csv'    converts data to a CSV file
            %      'Mat'    converts data to a MAT file
            %      'Raw'    returns the input path
            %
            %   'View' Parameter
            %      'Simple' converts only simple feedback. This is appropriate
            %               for most users and results in much smaller log
            %               files. (default)
            %      'Full'   converts all available feedback. This is appropriate
            %               for advanced users that need additional timestamps
            %               or data from less common sensors.
            %
            %    Example:
            %       % Load selected files and plot position feedback
            %       hebiLogs = HebiUtils.convertGroupLogsUI();
            %       HebiUtils.plotLogs(hebiLogs, 'position');
            %
            %    Example:
            %       % Load selected files as 'full' structs
            %       hebiLogs = HebiUtils.convertGroupLogsUI('View', 'full');
            %
            %    Example:
            %       % Get the filenames without any loading or conversion
            %       fileNames = HebiUtils.convertGroupLogsUI('format','raw');
            %
            %    Example:
            %       % Convert to .MAT file before loading
            %       % (This can be useful for very large logs)
            %       matFiles = HebiUtils.convertGroupLogsUI('format','mat','view','full');
            %       hebiLogs = cellfun(@load, matFiles, 'UniformOutput', false);
            %
            %    Example:
            %       % Add info/gains if available
            %       [hebiLogs, infos, gains] = HebiUtils.convertGroupLogsUI();
            %
            % See also HebiGroup.stopLog, convertGroupLog, loadGroupLogsUI.
            
            % Remember last file location
            persistent lastDir; 
            
            % Use current location if not set yet
            if isempty(lastDir)
                openFilePath = pwd;
            else
                openFilePath = lastDir;
            end

            % Show selector dialog
            [fileName,pathName] = uigetfile( '*.hebilog', ...
                '.hebilog Files (*.hebilog)', openFilePath,...
                'MultiSelect', 'on' );
            
            % Update current location if something was actually selected.
            if pathName ~= 0
                lastDir = pathName;
            end

            % Convert to cell array
            if ~iscell(fileName) && ~ischar(fileName)
                % No selection
                disp('No files chosen.');
                varargout = {{}};
                return;
            elseif ~iscell( fileName )
                % Single-select
                fileNames{1} = fileName;
            else
                % Multi-select
                fileNames = fileName;
            end
            
            % Load each log
            numLogs = length(fileNames);
            hebiLogs = cell(numLogs,1);
            infos = cell(numLogs,1);
            gains = cell(numLogs,1);
            for i=1:numLogs
                
                fullFileName = [pathName fileNames{i}];
                fileInfo = dir(fullFileName);
                
                disp([ 'Loading ' num2str(i) ' of ' ...
                    num2str(numLogs) ': '  fileNames{i} ' - ' ...
                    num2str(fileInfo.bytes/1E6,'%0.1f') ' MB']);
                
                if nargout <= 1
                    % pure conversion
                    hebiLogs{i} = ...
                        HebiUtils.convertGroupLog(fullFileName, ...
                        varargin{:});
                else
                    % also read info/gains
                    [hebiLogs{i}, infos{i}, gains{i}] = ...
                        HebiUtils.convertGroupLog(fullFileName, ...
                        varargin{:});
                end
                
            end
            
            % Output as 3 args, e.g., [hebiLogs, infos, gains] = ...
            varargout = {hebiLogs, infos, gains};
        end
        
        function [ ] = plotLogs( hebiLogs, feedbackField, varargin )
            %PLOTLOGS Nicely formatted plotting of HEBI logs.
            %
            %   HebiUtils.plotLogs(hebiLogs, feedbackField);
            %
            %   Required Arguments:
            %
            %       'InputLogs' is a single hebiLog object or a cell array 
            %       of log objects. Logs can come from logging modules 
            %       online with HebiGroup.stopLog(), loaded from a saved file  
            %       using HebiUtils.loadGroupLog(). Or selected manually 
            %       from a UI dialog using HebiUtils.loadGroupLogsUI().
            % 
            %       'FeedbackField' is a string corresponding to a feedback
            %       field in the log object. If the field is 'position',
            %       'velocity', or 'effort', the plots will show both the
            %       feedback and commanded values in an upper subplot, and
            %       it will show the error of the tracked commands in a
            %       lower subplot.
            %
            %   Optional Parameters:
            %
            %       'Modules' is a vector of module indices you would like
            %       to plot. If you do not provide this parameter or pass
            %       in [], then it will plot all modules in the group.
            %
            %       'FigNum' is the figure number you would like to use for
            %       plots. If multiple log files are plotted, then the
            %       subsequent figure numbers increment by 1 starting at
            %       FigNUm. If FigNum is not specified, then a new figure
            %       will be created to avoid over-writing figures.
            %
            %   Examples:
            %       % Plot the commanded and feedback positions of all the
            %       % modules in one or more logs.
            %       HebiUtils.plotLogs(hebiLogs, 'position');
            %
            %       % Same as above, except that this call only plots the
            %       % first two modules in the group, and the figure number
            %       % for multiple logs start at 100.
            %       HebiUtils.plotLogs(hebiLogs, 'position', ...
            %           'Modules', 1:2, ...
            %           'FigNum', 100);
            %
            % See also LOADGROUPLOG, LOADGROUPLOGSUI.
            
            if nargin < 1 || isempty(hebiLogs)
                disp('Please specify a log file and feedback field to plot.');
                return;
            end
            
            if nargin < 2 || isempty(feedbackField)
                disp('Please specify a feedback field (as a string) to plot.');
                return;
            end
            
            if ~iscell(hebiLogs)
                hebiLogs = {hebiLogs};
            end
            
            % Parse parameters
            parser = inputParser();
            parser.addParameter('Modules', [], @isnumeric);
            parser.addParameter('FigNum', [], @(a)~isempty(a));
            parser.CaseSensitive = false;
            parser.parse(varargin{:});
            p = parser.Results;
            
            % Check if we need a subplot, i.e., plot command & feedback
            isCommandPlot = strcmp(feedbackField,'position') || ...
                        strcmp(feedbackField,'velocity') || ...
                        strcmp(feedbackField,'effort');
            
            % Iterate thru each log and plot the desired feedback
            numLogs = length(hebiLogs);
            for i=1:length(hebiLogs)
                
                % Assign the mask for plotting only some modules from a
                % group.  If its empty figure out the size based on the
                % number of entries in the second field (the first is the
                % master 'time' vector that always has one).
                plotMask = p.Modules;
                if isempty(plotMask)
                    logFields = fields(hebiLogs{i});                 
                    plotMask = 1:size(hebiLogs{i}.(logFields{2}),2);
                end
                
                % Assign figure number, or count up from the user-defined
                % number
                if isempty(p.FigNum)
                    figure;
                else
                    figure(p.FigNum + i-1);
                    clf(p.FigNum + i-1);
                end
                
                % If pos/vel/effort, make a 2x1 subplot of the tracked
                % commanded and feedback values and tracking error.
                if isCommandPlot
                    ax = subplot(2,1,1);
                else
                    ax = axes;
                end
                
                % Clears old data if reusing a figure window
                if i==1
                    hold off;
                end
                
                plot(ax, hebiLogs{i}.time, hebiLogs{i}.(feedbackField)(:,plotMask) );
                
                xlabel('time (sec)');
                ylabel([feedbackField ' (' HebiUtils.getFeedbackUnits(feedbackField) ')']);
                title( [feedbackField ' - Log ' ...
                    num2str(i) ' of ' num2str(numLogs)] );
                xlim([0 hebiLogs{i}.time(end)]);
                grid on;
                
                % If pos/vel/effort, plot commands and error as well
                if isCommandPlot
                    hold on;
                    ax.ColorOrderIndex = 1;
                    plot( hebiLogs{i}.time, ...
                        hebiLogs{i}.([feedbackField 'Cmd'])(:,plotMask), '--' );
                    hold off;
                    
                    subplot(2,1,2);
                    
                    plot( hebiLogs{i}.time, ...
                        hebiLogs{i}.(feedbackField)(:,plotMask) - ...
                        hebiLogs{i}.([feedbackField 'Cmd'])(:,plotMask) );
                    
                    xlabel('time (sec)');
                    ylabel(['error (' HebiUtils.getFeedbackUnits(feedbackField) ')']);
                    title( [feedbackField ' error'] );
                    xlim([0 hebiLogs{i}.time(end)]);
                    grid on;
                end
                
                % Add legend at the end so that it doesn't generate  
                % 'dataN' values for command lines.
                legend(strsplit(num2str(plotMask)));
                
            end
        end
        
        function [ fig ] = plotTrajectory( varargin )
            %PLOTTRAJECTORY Visualize the output of HebiTrajectory.
            %
            %   This method visualizes position, velocity, and acceleration
            %   of a HebiTrajectory.
            %
            %   Arguments:
            %
            %       trajectory  - HebiTrajectory that should be plotted
            %
            %   Parameters:
            %
            %       'dt' is the timestep used for plotting, in seconds. The
            %       default value if left unspecified is 0.01 seconds.
            %
            %       'FigNum' is the figure number or figure handle that
            %       should be used for plotting. If a figure with the
            %       specified number exists it will be overwritten. If left
            %       unspecified, a new figure will automatically be
            %       generated.
            %
            %       'Legend' is a string or cell array of the text that gets
            %       displayed as the legend. By default it shows the joint
            %       number.
            %
            %   Examples:
            %       % Visualize trajectory moving through 5 random waypoints
            %       velocityLimit = [-10 10];
            %       trajGen = HebiTrajectoryGenerator(velocityLimit);
            %       trajGen.setMinDuration(0.1);
            %       positions = 2*pi * rand(5,1);
            %
            %       traj = trajGen.newJointMove(positions);
            %       figHandle = HebiUtils.plotTrajectory(traj,'FigNum',101);
            %
            %   See also HEBITRAJECTORYGENERATOR, HEBITRAJECTORY.
            
            % Parse arguments
            parser = inputParser();
            parser.addRequired('Trajectory', @(a) isa(a, 'HebiTrajectory'));
            parser.addParameter('dt', 0.01, @isnumeric);
            parser.addParameter('FigNum', [], @(a)~isempty(a));
            parser.addParameter('Legend', [], @(a)iscell(a)|ischar(a));
            parser.CaseSensitive = false;
            parser.parse(varargin{:});
            p = parser.Results;
            
            % Select figure
            if isempty(p.FigNum)
                fig = figure;
            else
                fig = figure(p.FigNum);
            end
            
            trajectory = p.Trajectory;
            time = 0:p.dt:trajectory.getDuration();
            waypointTime = trajectory.getWaypointTime();
            
            [pos,vel,acc] = trajectory.getState(time);
            [wayPos,wayVel,wayAcc] = trajectory.getState(waypointTime);
            
            ax = subplot(3,1,1);
            plot(time,pos);
            hold on;
            ax.ColorOrderIndex = 1;
            plot(waypointTime,wayPos,'o');
            hold off;
            title('Trajectory Profile');
            ylabel('position (rad)');
            xlabel('time (sec)');
            grid on;
            
            if isempty(p.Legend)
                legend(strsplit(num2str(1:size(pos,2))));
            else
                legend(p.Legend);
            end
            
            ax = subplot(3,1,2);
            plot(time,vel);
            hold on;
            ax.ColorOrderIndex = 1;
            plot(waypointTime,wayVel,'o');
            hold off;
            ylabel('velocity (rad/sec)');
            xlabel('time (sec)');
            grid on;
            
            ax = subplot(3,1,3);
            plot(time,acc);
            hold on;
            ax.ColorOrderIndex = 1;
            plot(waypointTime,wayAcc,'o');
            hold off;
            ylabel('acceleration (rad/sec^2)');
            xlabel('time (sec)');
            grid on;
            
        end
        
        function [ R ] = quat2rotMat( q )
            %QUAT2DCM Conversion of a quaternion to an orthogonal rotation matrix.
            %Assumes that the scalar element, q_w, is the first element of the 
            %quaternion vector, q = [q_w q_x q_y q_z].
            %
            %   R = quat2rotMat( q )
            %
            %   If q needs to be a [4x1] row vector. R is a [3x3] SO3 rotation 
            %   matrix. You can batch process N quaternions by passing in a [Nx4] 
            %   matrix where each row is a quaternion. In this case thefunction 
            %   will return N rotation matrices of size [3x3xN].
            
            num_q = size(q,1);
            R = zeros(3,3,num_q);

            for i=1:num_q

                a = q(i,2);
                b = q(i,3);
                c = q(i,4);
                d = -q(i,1);

                R(:,:,i) = [ a^2 - b^2 - c^2 + d^2,   2*(a*b + c*d),       2*(a*c - b*d);
                             2*(a*b - c*d),   -a^2 + b^2 - c^2 + d^2,    2*(b*c + a*d);
                             2*(a*c + b*d),         2*(b*c - a*d),  -a^2 - b^2 + c^2 + d^2];
            end    
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
    
    % Experimental utility methods for internal use. May be made part of
    % the public API at some point or may be removed without notice.
    methods(Access = public, Static, Hidden = true)
        
        function [ feedbackUnits ] = getFeedbackUnits( feedbackField )
            %FEEDBACKUNITS Return units of a given feedback type in a log file
            switch feedbackField
                
                % Actuator Feedback
                case {'position','positionCmd','motorPosition','deflection'}
                    feedbackUnits = 'rad';
                    
                case {'velocity','velocityCmd','motorVelocity','deflectionVelocity'}
                    feedbackUnits = 'rad/sec';
                    
                case {'effort','effortCmd','innerEffortCmd'}
                    feedbackUnits = 'Nm';
                    
                case {'time','pcRxTime','pcTxTime','hwRxTime','hwTxTime'}
                    feedbackUnits = 'sec';
                    
                case {'pwmCmd'}
                    feedbackUnits = '-1 to 1';
                    
                case {'accelX','accelY','accelZ'}
                    feedbackUnits = 'm/sec^2';
                    
                case {'gyroX','gyroY','gyroZ'}
                    feedbackUnits = 'rad/sec';
                    
                case {'motorTemperature','windingTemperature','ambientTemperature'...
                        'processorTemperature','actuatorTemperature'}
                    feedbackUnits = 'deg-C';
                    
                case {'motorCurrent','windingCurrent'}
                    feedbackUnits = 'A';
                    
                case {'voltage'}
                    feedbackUnits = 'V';
                    
                case {'ledR','ledRG','ledB'}
                    feedbackUnits = '0-1';
                
                % Mobile Feedback
                case {'gpsTimestamp'}
                    feedbackUnits = 'sec';
                    
                case {'magnetometerX','magnetometerY','magnetometerZ'}
                    feedbackUnits = 'T';
                    
                case {'gpsLatitude','gpsLongitude','gpsHeading'}
                    feedbackUnits = 'deg';    
               
                case {'altitude','gpsAltitude','gpsHorizontalAccuracy','gpsVerticalAccuracy'}
                    feedbackUnits = 'm';
                    
                case {'arPositionX','arPositionY','arPositionZ'}
                    feedbackUnits = 'm';    
                    
                case {'batteryLevel'}
                    feedbackUnits = '%';
                    
                otherwise
                    feedbackUnits = '';
            end
        end
     
        
        function [ R ] = axAng2rotMat( axis, angle )
            %AXANG2ROTMAT Convert an orientation represented in axis-angle form into an
            %SO3 rotation matrix.  AXIS is a unit vector, and ANGLE is between +/- pi
            %radians.  R is a 3X3 S03 rotation matrix.
            %
            %   Based on wikipedia:
            %   https://en.wikipedia.org/wiki/Rotation_matrix#Axis_and_angle
            
            axis = axis / norm(axis);
            
            c = cos(angle);
            s = sin(angle);
            C = 1-c;
            
            x = axis(1);
            y = axis(2);
            z = axis(3);
            
            R = [  x*x*C + c   x*y*C - z*s   x*z*C + y*s;
                y*x*C + z*s   y*y*C + c    y*z*C - x*s;
                z*x*C - y*s  z*y*C + x*s    z*z*C + c ];      
        end
        
        function [ axis, angle ] = rotMat2axAng( R )
            %ROTMAT2AXANG Convert an rotation matrix (SO3) to axis-angle
            %representation.  AXIS is a unit vector, and ANGLE is between +/- pi
            %radians.  R is a 3X3 S03 rotation matrix.
            %
            %   Based on StackExchange question:
            %   https://math.stackexchange.com/questions/2217654/interpolation-in-so3-different-approaches
            
            [eigVec,eigDiag] = eig(R);
            eigVal = diag(eigDiag);
            
            [~,maxIdx] = max(real(eigVal));
            
            angle = acos((trace(R)-1)/2);
            axis = real(eigVec(:,maxIdx));
            
            R_check = HebiUtils.axAng2rotMat(axis,angle);
            
            checkTolerance = 1E-6;
            if max(max( abs(R_check*R'-eye(3)) )) > checkTolerance
                angle = -angle;
            end
            
            % Keep angle between +/- pi
            if abs(angle) > pi
                angle = sign(angle)*(abs(angle) - 2*pi);
            end            
        end
        
    end
    
end
