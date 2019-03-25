classdef (Sealed) HebiGroup < handle
    % HebiGroup is a collection of one or more modules.
    %
    %   Groups of modules are the basic way to send commands and retrieve
    %   feedback. They provide convenient ways to deal with modules, and
    %   handle high-level issues such as data synchronization and logging.
    %
    %   HebiGroup Methods (configuration):
    %   getNumModules        - returns the number of grouped modules
    %   setFeedbackFrequency - sets the feedback request rate
    %   getFeedbackFrequency - gets the feedback request rate
    %   setCommandLifetime   - sets the command lifetime
    %   getCommandLifetime   - gets the command lifetime
    %
    %   HebiGroup Methods (common):
    %   send                  - sends synchronized commands, as well as a
    %                           number of other settings and parameters
    %   getNextFeedback       - returns the next new 'simple' feedback
    %   getNextFeedbackFull   - returns the next new 'full' feedback  
    %   getNextFeedbackIO     - returns the next new 'I/O' board feedback 
    %   getNextFeedbackMobile - returns the next new 'mobile' feedback 
    %   getInfo               - returns meta information such as names
    %   getGains              - returns the current gains
    %   startLog              - starts background logging to disk
    %   stopLog               - stops logging and returns a readable format
    %   stopLogFull           - same as above, returning full feedback
    %   stopLogIO             - same as above, returning I/O feedback
    %   stopLogMobile         - same as above, returning mobile feedback
    %
    %   Example
    %      % 200 Hz loop commanding of the current position
    %      group.setFeedbackFrequency(200);
    %      cmd = CommandStruct();
    %      group.startLog();
    %      t0 = tic();
    %      while toc(t0) < 5 
    %          fbk = group.getNextFeedback();
    %          cmd.position = fbk.position;
    %          group.send(cmd);
    %      end
    %      log = group.stopLog();
    %
    %      % Plotting the commanded / feedback positions
    %      plot(log.time, log.position);
    %      hold on;
    %      plot(log.time, log.positionCmd, '--');
    %      hold off;
    %      xlabel('time (sec)');
    %      ylabel('position (rad)');
    %      legend(group.getInfo.name); 
    %
    %   See also HebiLookup, send, getInfo, startLog, getNextFeedback.
    
    %   Copyright 2014-2018 HEBI Robotics, Inc.
    
    % Public API
    methods(Access = public)
        
        function out = getNumModules(this, varargin)
            %getNumModules  returns the number of modules within a group.
            %
            %   Example
            %      % Command all-zero efforts
            %      cmd = CommandStruct()
            %      cmd.effort = zeros(1, group.getNumModules())
            %      group.send(cmd);
            % 
            %   See also HebiLookup, getInfo.
            out = getNumModules(this.obj, varargin{:});
        end
        
        function out = getFeedbackFrequency(this, varargin)
            %getFeedbackFrequency returns the feedback polling frequency [Hz]
            %
            %   getFeedbackFrequency() returns the request rate in Hz for
            %   feedback a group. 
            %
            %   The default feedback frequency for a new group is 100 Hz. A 
            %   rate of 0.0 indicates that no feedback is being requested.
            %
            %   Note that the scheduler will try it's best effort to reach
            %   the set frequency, but the actual arriving rate may be 
            %   lower as it is limited by the underlying operating system 
            %   as well as potential hardware constraints.
            %
            %   Example
            %      % Estimate the 'real' incoming feedback frequency
            %      targetFrequency = 100; % [Hz]
            %      pauseTime = 10; % [s]
            %      group.setFeedbackFrequency(targetFrequency);
            %      group.startLog();
            %      pause(pauseTime);
            %      log = group.stopLog();
            %      realFrequency = length(log.time) / pauseTime;
            %
            %   See also HebiGroup, setFeedbackFrequency
            out = getFeedbackFrequency(this.obj, varargin{:});
        end
        
        function this = setFeedbackFrequency(this, varargin)
            %setFeedbackFrequency sets the feedback polling frequency [Hz]
            %
            %   setFeedbackFrequency(frequency) sets the target frequency 
            %   in Hz at which feedback requests get sent out. 
            %
            %   The default feedback frequency for a new group is 100 Hz. A 
            %   rate of 0.0 stops sending outgoing feedback requests.
            %
            %   Note that the scheduler will try it's best effort to reach
            %   the set frequency, but the actual arriving rate may be 
            %   lower as it is limited by the underlying operating system 
            %   as well as potential hardware constraints.
            %
            %   Example
            %      % Estimate the 'real' incoming feedback frequency
            %      targetFrequency = 100; % [Hz]
            %      pauseTime = 10; % [s]
            %      group.setFeedbackFrequency(targetFrequency);
            %      group.startLog();
            %      pause(pauseTime);
            %      log = group.stopLog();
            %      realFrequency = length(log.time) / pauseTime;
            %
            %   See also HebiGroup, getFeedbackFrequency
            setFeedbackFrequency(this.obj, varargin{:});
        end
        
        function out = getCommandLifetime(this, varargin)
            %getCommandLifetime returns the current command lifetime [s].
            %
            %   This method provides a way to programmatically access 
            %   the current command lifetime.
            %
            %   The command lifetime is the duration for which a sent 
            %   command remains active. If the hardware does not receive
            %   further commands within the specified time frame, all local
            %   controllers get deactivated. This is a safety feature
            %   to mitigate the risk of accidents in case programs get
            %   interrupted in an unsafe state, e.g., on user interrupts
            %   (ctrl+c) or during a network fault.
            %
            %   Additionally, supporting hardware does not accept commands
            %   from any other sources during the lifetime of a command.
            %   This mitigates the risk of other users accidentally sending
            %   conflicting targets from, e.g., the GUI, or any other groups
            %   running in Matlab or from any other APIs.
            %
            %   The default command lifetime for a new group is 0.25 sec.
            %
            %   See also HebiGroup, setCommandLifetime
            out = getCommandLifetime(this.obj, varargin{:});
        end
        
        function this = setCommandLifetime(this, varargin)
            %setCommandLifetime sets the command lifetime [s].
            %
            %   The command lifetime is the duration for which a sent 
            %   command remains active. If the hardware does not receive
            %   further commands within the specified time frame, all local
            %   controllers get deactivated. This is a safety feature
            %   to mitigate the risk of accidents in case programs get
            %   interrupted in an unsafe state, e.g., on user interrupts
            %   (ctrl+c) or during a network fault.
            %
            %   Additionally, supporting hardware does not accept commands
            %   from any other sources during the lifetime of a command.
            %   This mitigates the risk of other users accidentally sending
            %   conflicting targets from, e.g., the GUI, or any other groups
            %   running in Matlab or from any other APIs.
            %
            %   This feature can be disabled by setting 'inf' or the empty
            %   matrix '[]'. When disabled, the hardware will continue to
            %   execute the last sent command. Note that this can result in
            %   unexpected behavior when sending efforts and velocities.
            %
            %   The default command lifetime for a new group is 0.25 sec.
            %
            %   Example
            %      % Stop motions if hardware does not receive commands at
            %      % least once every 100 ms.
            %      group.setCommandLifetime(0.1);
            %
            %   See also HebiGroup, getCommandLifetime
            setCommandLifetime(this.obj, varargin{:});
        end
        
        function out = send(this, varargin)
            %send sends commands and settings to modules.
            %
            %   This method provides a variety of selectors to send commands, gains,
            %   LEDs, as well as a several useful settings and flags. This method returns
            %   immediately and does not wait until outgoing packets have arrived at
            %   the receiving modules.
            %
            %   'Boot' boots a module from bootloader mode into application
            %   mode. It expects a boolean value (true will boot) and can
            %   only be used in supported modes (e.g. bootloader mode).
            %
            %   'Command' ('cmd') is used to send common control loop set points
            %   such as positions, velocities, or efforts. It expects a CommandStruct.
            %   If struct fields are empty or filled with NaN, the underlying control
            %   is disabled. For the common use case (set CommandStruct), the flag can
            %   be omitted (both valid use cases are shown below).
            %
            %   Example
            %      % Enable/disable velocity control
            %      cmd = CommandStruct();
            %      cmd.velocity = zeros(1, group.getNumModules);
            %      t0 = tic;
            %      while toc(t0) < 5
            %          fbk = group.getNextFeedback: % limit loop rate
            %          group.send('cmd', cmd); 
            %      end 
            %      cmd.velocity = [];
            %      group.send(cmd);
            %
            %   'IoCommand' ('IoCmd') is used to send pin commands to supporting
            %   modules, e.g., I/O boards. It expects an IoCommandStruct. Fields that
            %   are empty or contain NaN/Inf values will not be sent. For the common
            %   use case (set IoCommandStruct), the flag can be omitted.
            %
            %   Example
            %      % map pins (improves readability)
            %      selectedPin = 'e1';
            %      % set digital pin to 1
            %      ioCmd = IoCommandStruct();
            %      ioCmd.(selectedPin) = 1;
            %      group.send(ioCmd);
            %
            %   'Gains' sets control loop gains. Note that there are many tuning
            %   parameters that depend on the used control strategy. If you need
            %   to reset gains, you can simply reboot a module to restore the
            %   previously persisted gains.
            %
            %   Example
            %      % set effort limits of +/- 1.5 Nm|N
            %      n = group.getNumModules();
            %      limit = 1.5;
            %      gains = GainStruct();
            %      gains.effortMaxOutput = ones(1,n) * limit;
            %      gains.effortMinOutput = ones(1,n) * -limit;
            %      group.send('gains', gains);
            %
            %   'LED' sets the led color. This is often useful when synchronizing video
            %   to feedback and for timing analysis in combination with a high-speed
            %   camera. Colors can be set identically for all modules, or individually
            %   for each module. The string mappings can be found in 'help plot'.
            %
            %   Example
            %      group.send('led', 'red'); % set all to red
            %      group.send('led', [0 0 1]); % set all to blue [r g b] [0-1]
            %      group.send('led', []); % reset to default mode
            %
            %   'Name' sets the names for individual modules. It expects a {1xN cell}
            %   array of module names, where N is the number of modules in a group.
            %   Valid names are limited to alphanumerical characters, spaces, and
            %   underscores.
            %
            %   'Family' sets the family for all or individual modules. It expects a
            %   single string or an {1xN cell} array of strings. Valid families are
            %   limited to alphanumerical characters, spaces, and underscores.
            %
            %   'Persist' saves all the settings and gains that are currently set on 
            %   on a module, so that they are loaded after reboot.  It expects a 
            %   boolean value. True will save settings, false has no effect.
            %
            %   'Reset' reboots a module. It expects a boolean value and can only be
            %   used in supported modes (e.g. application mode). True will reboot a
            %   module, false has no effect.
            %
            %   'PositionLimit' ('PosLim') sets safety limits for position.
            %   Safety limits act as a virtual hard stop and are independent of gains.
            %
            %   'VelocityLimit' ('VelLim') sets safety limits for velocity.
            %   Safety limits act as a virtual hard stop and are independent of gains.
            %
            %   'EffortLimit' ('EffLim') sets safety limits for effort.
            %   Safety limits act as a virtual hard stop and are independent of gains.
            %
            %   'ReferencePosition' sets the current position (feedback) by adjusting
            %   the user-settable reference point for the zero position. This persists
            %   automatically. This is the same as setting the reference point for
            %   Position in the 'Device' tab of the Scope GUI.
            %
            %   'OffsetReferencePosition' offsets the current position (feedback) by
            %   offsetting the user-settable reference point for the zero position.
            %   
            %   'ZeroReferenceDeflection' sets the deflection of the internal spring
            %   to zero. Note that this will also cause the effort (feedback) to also be
            %   zero. Only do this when the actuator is unloaded.
            %
            %   'OffsetReferenceDeflection' adjusts the internal spring deflection. Note
            %   that this will cause a jump in the effort feedback that depends on the
            %   spring constant (may or may not be linear).
            %
            %   'SpringConstant' sets the internal stiffness parameter for an actuator
            %   that is used to turn its sensed spring deflection into an estimated 
            %   effort (torque). This is a linear spring constant, units are Nm/rad. 
            %   The current spring constant can be determined by getting a 'full' 
            %   feedback and dividing the negative effort by the measured deflection 
            %   (-fbk.effort ./ fbk.deflection).
            %
            %   'RequestAck' ('Ack') requests message acknowledgements from each 
            %   device. This method will return true if acknowledgements have been
            %   received from all devices within this group and within the specified
            %   timeout.
            %   
            %   'Timeout' [s] the deadline for receiving acknowledgements before
            %   this method aborts and returns false. (Default 0.5s)
            %
            %   Note that all options can be combined as required. Options that get set
            %   in the same function call will be packed into the same outgoing packet.
            %
            %   Example
            %      % Indicate with led that family has been set
            %      group.send('family', 'MyRobot', 'led', 'r');
            %
            %   See also HebiGroup, CommandStruct, GainStruct
            out = send(this.obj, varargin{:});
        end
              
        function out = get(this, varargin)
            %get returns a variety of data structs
            %
            %   This method provides a variety of selectors to return info, feedback,
            %   or gain data in a variety of formats. It always returns immediately
            %   and may return the same data more than once. In practice you should
            %   use an appropriate convenience wrapper.
            %
            %   See also
            %      getInfo
            %      getGains
            %      getNextFeedback
            %      getNextFeedbackFull
            %      getNextFeedbackIO
            %      getNextFeedbackMobile
            %
            %   If a struct returns empty or full of NaN, make sure that the
            %   appropriate polling rate has been set.
            %
            %   Example
            %      % Request info/gains at 5Hz and feedback at 100Hz
            %      HebiLookup.setLookupFrequency(5);
            %      group.setFeedbackFrequency(100);
            %
            %   DataType argument
            %      'Info'     returns meta information about the module, such
            %                 as versions, addresses, names, etc.
            %      'Gains'    returns the gains for the active control strategy.
            %                 Gains that are disabled, are set to NaN.
            %      'Feedback' returns aggregated sensor feedback
            %
            %   'View' Parameter ('Feedback' only)
            %      'Simple'   returns basic feedback. This is appropriate for most
            %                 users. (default)
            %      'Full'     returns all available feedback. This is appropriate
            %                 for advanced users that care about additional
            %                 timestamps and less common sensors.
            %      'IO'       returns feedback fields specific to I/O devices,
            %                 such as analog and digital pins
            %      'Mobile'   returns feedback fields specific to mobile devices,
            %                 such as GPS and ARKit/ARCore estimates.
            %
            %   See also HebiGroup
            out = get(this.obj, varargin{:});
        end
        
        function out = getNextFeedback(this, varargin)
            %getNextFeedback returns the next new synchronized feedback.
            %
            %   This method returns the next available feedback. This function
            %   ensures that feedback will not get returned multiple times. If there
            %   already is new feedback available, then this function returns
            %   immediately. If no feedback is available, this function waits until
            %   new feedback arrives or a timeout is reached.
            %
            %   If this function reaches the timeout, it will either throw an error
            %   (live groups) or return an empty matrix (log replay groups). If this
            %   happens, make sure that the hardware is turned on and that the feedback
            %   request rate is set appropriately.
            %
            %   'Timeout' Parameter sets the timeout in seconds.
            %
            %   'View' Parameter
            %      'Simple'   returns basic feedback. This is appropriate for most
            %                 users. (default)
            %      'Full'     returns all available feedback. This is appropriate
            %                 for advanced users that care about additional
            %                 timestamps and less common sensors.
            %      'IO'       returns the state of pins on an IO board.
            %
            %   Example
            %      % retrieve feedback from group
            %      group = HebiLookup.newGroupFromFamily('*');
            %      group.setFeedbackFrequency(100);
            %      simpleFbk = group.getNextFeedback();
            %      fullFbk = group.getNextFeedback('View', 'full');
            %
            %   Note that creating a feedback struct can be expensive in some time
            %   critical applications. For such cases, there is a way to reuse
            %   existing structs.
            %
            %   Example
            %      % Reuse struct using standard syntax
            %      reuseStruct = group.getFeedbackFull(); % create once
            %      while true % reuse in loop
            %          fbk = group.getNextFeedback(reuseStruct);
            %          display(fbk.position);
            %      end
            %
            %      % More optimized syntax
            %      fbk = group.getNextFeedback();
            %      while ~isempty(getNextFeedback(group, fbk))
            %          display(fbk.position);
            %      end
            %
            %   See also HebiGroup, getFeedbackFrequency, getNextFeedbackFull,
            %   getNextFeedbackIO., getNextFeedbackMobile
            out = getNextFeedback(this.obj, varargin{:});
        end
        
        function out = getNextFeedbackFull(this, varargin)
            %getNextFeedbackFull is a convenience wrapper for getNextFeedback
            %
            %   This method is a convenience wrapper with the same behavior as
            %   calling getNextFeedback('View', 'Full'). The 'Full' view
            %   offers additional sensor feedback such as hardware
            %   timestamps and less common sensor measurements.
            %
            %   Example
            %      % Find the network round trip time
            %      fbk = group.getNextFeedbackFull()
            %      roundTripTime = fbk.pcRxTime - fbk.pcTxTime;
            %
            %   See also HebiGroup, getNextFeedback.
            out = getNextFeedbackFull(this.obj, varargin{:});
        end

        function out = getNextFeedbackIO(this, varargin)
            %getNextFeedbackIO is a convenience wrapper for getNextFeedback
            %
            %   This method is a convenience wrapper with the same behavior as
            %   calling getNextFeedback('View', 'IO'). The 'IO' view provides
            %   access to the state of pins on an I/O board, as well as
            %   hardware timestamps.
            %
            %   Example
            %      % Read the value of pin A1
            %      fbk = group.getNextFeedbackIO()
            %      value = fbk.a1;
            %
            %   See also HebiGroup, getNextFeedback.
            out = getNextFeedbackIO(this.obj, varargin{:});
        end
        
        function out = getNextFeedbackMobile(this, varargin)
            %getNextFeedbackMobile is a convenience wrapper for getNextFeedback
            %
            %   This method is a convenience wrapper with the same behavior as
            %   calling getNextFeedback('View', 'Mobile'). The 'Mobile' view 
            %   provides access to data from mobile devices such as phones
            %   or tablets. This includes fields such as GPS data, and 
            %   ARCore/ARKit estimates.
            %
            %   Example
            %      % Read the current battery level (in %)
            %      fbk = group.getNextFeedbackMobile()
            %      value = fbk.batteryLevel;
            %
            %   See also HebiGroup, getNextFeedback.
            out = getNextFeedbackMobile(this.obj, varargin{:});
        end
        
        function out = getGains(this, varargin)
            %getGains returns the current gains
            %
            %   This method returns the latest received gains. It returns
            %   immediately and may return the same result more than once.
            %   Note that gain structs get populated by lookup requests,
            %   which usually get requested at a lower rate than feedback.
            %   If gain info is not available, this method will return
            %   empty ([]).
            %
            %   If gains return empty or do not update as expected, confirm
            %   that the lookup polling rate is larger than zero.
            %
            %   Example
            %      % Display the lookup polling rate
            %      HebiLookup
            %
            %   Example
            %      % Double position kp gains on all modules
            %      gains = group.getGains()
            %      gains.positionKp = gains.positionKp * 2;
            %      group.send('gains', gains);
            %
            %   See also HebiGroup, HebiLookup.setLookupFrequency.
            out = getGains(this.obj, varargin{:});
        end
        
        function out = getInfo(this, varargin)
            %getInfo returns meta information such as names
            %
            %   This method returns a table of the latest received info.
            %   It returns immediately and may return the same result more than
            %   once. Note that info data gets populated by lookup requests,
            %   which usually get requested at a lower rate than feedback.
            %   If info is not available, this method returns empty ([]).
            %
            %   The info table contains meta information about modules, such
            %   as their names, versions, network settings, and serial numbers.
            %
            %   Example
            %      % Retrieve module names
            %      info = group.getInfo();
            %      names = info.name;
            %
            %   Example
            %      % Check age of received info
            %      info = group.getInfo();
            %      age = HebiUtils.now() - info.pcRxTime;
            %
            %   Example
            %      % Display the lookup polling rate
            %      HebiLookup
            %
            %   See also HebiGroup, HebiLookup.setLookupFrequency.
            out = getInfo(this.obj, varargin{:});
            if ~isempty(out) % convert struct to table
                out = struct(out);
                out = rmfield(out, {'time', 'numModules'}); % remove scalars
                out = struct2table(out);
                if size(out, 1) > 1
                    % clear null row
                    out(end, :) = [];
                end
            end
        end
        
        function out = startLog(this, varargin)
            %startLog logs incoming feedback data to disk
            %
            %   This method starts logging of incoming module feedback.
            %   Logging is done in the background and does not interfere
            %   with ongoing operations.
            %
            %   Note that all logs are initially streamed into a binary
            %   streaming format, which gets converted into a readable
            %   format by stopLog. Alternatively, binary logs can also be
            %   converted manually by convertGroupLog, which can be useful
            %   after a system crash or if different formats are needed.
            %
            %   If the log is empty, make sure that the feedback polling
            %   rate is set and that the hardware is turned on.
            %
            %   A repeated call to startLog will stop active logging, and
            %   restart in a new file. No data will be lost. Doing this
            %   has little overhead, and can be useful to split log-files
            %   in a long running application.
            %
            %   'Directory' ('dir') Parameter
            %      '<name>'  sets the relative or absolute directory of the
            %                binary file
            %      (default) current working directory
            %
            %
            %   'FileName' ('file') Parameter
            %      '<name>'  sets the filename for the binary file, as well as
            %                any later converted file such as mat or csv.
            %      (default) a generated unique name based on the timestamp
            %
            %   Example
            %       group.startLog();
            %       pause(1);
            %       log = group.stopLog();
            %       plot(log.time, log.position);
            %
            %   See also HebiGroup, stopLog, stopLogFull, stopLogIO,
            %   stopLogMobile, HebiUtils.convertGroupLog.
            out = startLog(this.obj, varargin{:});
        end
        
        function out = stopLog(this, varargin)
            %stopLog stops logging and converts the log to a specified format
            %
            %   This method stops logging, converts the binary streaming
            %   format into a specified format, and returns an appropriate
            %   return value.
            %
            %   If the result is a struct, this method returns the struct.
            %
            %   If the result is a file, this method returns the absolute
            %   path to the resulting file.
            %
            %   'LogFormat' ('format') Parameter
            %      'Memory' converts to an in-memory struct. Large logs can
            %               result in out-of-memory errors. (default)
            %      'Csv'    converts data to a CSV file
            %      'Mat'    converts data to a MAT file
            %      'Raw'    returns the path to the raw .hebilog file and 
            %               does not perform any format conversion. This
            %               is useful when you do not want there to be any
            %               pauses or hangs due to log conversion.
            %
            %   'View' Parameter
            %      'Simple' converts only simple feedback. This is appropriate
            %               for most users and results in much smaller log
            %               files. (default)
            %      'Full'   converts all available feedback. This is appropriate
            %               for advanced users that need additional timestamps
            %               or data from less common sensors.
            %      'IO'     converts feedback related to input/output
            %               devices.
            %      'Mobile' converts feedback related to mobile devices
            %               such as phones or tablets.
            %
            %   Example
            %       % Log into .mat file and load into MATLAB
            %       group.startLog();
            %       pause(1);
            %       log = load(group.stopLog('LogFormat', 'mat'));
            %       plot(log.time, log.position);
            %
            %    Example
            %       % Plot positions over time
            %       group.startLog();
            %       pause(5);
            %       log = group.stopLogFull();
            %       figure()
            %       hold on;
            %       plot(time, log.position, '-');
            %       plot(time, log.positionCmd, '--');
            %       ylabel('position [rad]');
            %       xlabel('time [s]');
            %       title('Logged Positions')
            %
            %   See also HebiGroup, startLog, stopLogFull, stopLogIO.
            out = stopLog(this.obj, varargin{:});
            
            % stop log should only be called in non-time critical sections,
            % so it may make sense to do some automated cleanup.
            if HebiGroup.config.triggerStopLogGC
                java.lang.System.gc();
            end
        end
        
        function out = stopLogFull(this, varargin)
            %stopLogFull is a convenience wrapper for stopLog
            %
            %   This method is a convenience wrapper with the same behavior
            %   as calling stopLog('View', 'Full'). The 'Full' view
            %   offers additional sensor feedback such as hardware
            %   timestamps and less common sensor measurements.
            %
            %   See also HebiGroup, stopLog
            out = stopLogFull(this.obj, varargin{:});
            
            % stop log should only be called in non-time critical sections,
            % so it may make sense to do some automated cleanup.
            if HebiGroup.config.triggerStopLogGC
                java.lang.System.gc();
            end
            
        end

        function out = stopLogIO(this, varargin)
            %stopLogIO is a convenience wrapper for stopLog
            %
            %   This method is a convenience wrapper with the same behavior
            %   as calling stopLog('View', 'IO'). The 'IO' view provides
            %   access to the state of pins on an I/O board, as well as
            %   hardware timestamps.
            %
            %   See also HebiGroup, stopLog.
            out = stopLogIO(this.obj, varargin{:});

            % stop log should only be called in non-time critical sections,
            % so it may make sense to do some automated cleanup.
            if HebiGroup.config.triggerStopLogGC
                java.lang.System.gc();
            end

        end
        
         function out = stopLogMobile(this, varargin)
            %stopLogMobile is a convenience wrapper for stopLog
            %
            %   This method is a convenience wrapper with the same behavior
            %   as calling stopLog('View', 'Mobile'). The 'Mobile' view 
            %   provides access to data from mobile devices such as phones
            %   or tablets. This includes fields such as GPS data, and 
            %   ARCore/ARKit estimates.
            %
            %   See also HebiGroup, stopLog.
            out = stopLogMobile(this.obj, varargin{:});

            % stop log should only be called in non-time critical sections,
            % so it may make sense to do some automated cleanup.
            if HebiGroup.config.triggerStopLogGC
                java.lang.System.gc();
            end

        end
        
    end
    
    methods(Access = public, Hidden = true)
        
        function this = HebiGroup(obj)
            % constructor
            switch(nargin)
                case 0
                    % default constructor
                    this.obj = javaObject(HebiGroup.className);
                case 1
                    % wrapper
                    if(~isa(obj, HebiGroup.className))
                        error('invalid argument');
                    end
                    this.obj = obj;
            end
        end
        
        function [] = delete(this)
            %destructor disposes this instance
            delete(this.obj);
        end
        
        function disp(this)
            %custom display
            disp(this.obj);
        end
        
        function [] = set(this, varargin)
            %set forwards to 'send' and exists for backwards compatibility
            %
            %   See also send
            send(this.obj, varargin{:});
        end
        
        % Inherited methods that we don't want to see in
        % auto-complete or docs
        function varargout = addlistener(varargin)
            varargout{:} = addlistener@handle(varargin{:});
        end
        function varargout = eq(varargin)
            varargout{:} = eq@handle(varargin{:});
        end
        function varargout = findobj(varargin)
            varargout{:} = findobj@handle(varargin{:});
        end
        function varargout = findprop(varargin)
            varargout{:} = findprop@handle(varargin{:});
        end
        function varargout = ge(varargin)
            varargout{:} = ge@handle(varargin{:});
        end
        function varargout = gt(varargin)
            varargout{:} = gt@handle(varargin{:});
        end
        function varargout = le(varargin)
            varargout{:} = le@handle(varargin{:});
        end
        function varargout = listener(varargin)
            varargout{:} = listener@handle(varargin{:});
        end
        function varargout = lt(varargin)
            varargout{:} = lt@handle(varargin{:});
        end
        function varargout = ne(varargin)
            varargout{:} = ne@handle(varargin{:});
        end
        function varargout = notify(varargin)
            varargout{:} = notify@handle(varargin{:});
        end
        
    end
    
    properties(Access = private, Hidden = true)
        obj
    end
    
    properties(Constant, Access = private, Hidden = true)
        className = hebi_load('HebiGroup');
        config = hebi_config('HebiGroup');
    end
    
    % Non-API Static methods for MATLAB compliance
    methods(Access = public, Static, Hidden = true)
        
        function varargout = methods(varargin)
            instance = javaObject(HebiGroup.className);
            switch nargout
                case 0
                    methods(instance, varargin{:});
                    varargout = {};
                otherwise
                    varargout{:} = methods(instance, varargin{:});
            end
        end
        
        function varargout = fields(varargin)
            instance = javaObject(HebiGroup.className);
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
