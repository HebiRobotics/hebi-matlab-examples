classdef HebiMobileIO < handle
    % HebiMobileIO is utility wrapper for the HEBI Robotics mobileIO phone app
    %   
    %   HebiMobileIO Methods:
    %
    %   findDevice         - repeatedly searches the network until the device is found
    %
    %   initializeUI       - initializes all UI elements to their default state
    %   setAxisSnap        - sets the axis snap position
    %   setAxisValue       - sets the axis position
    %   setButtonToggle    - sets the button toggle mode
    %   setButtonIndicator - sets a visual indicator around a button
    %   addText            - appends a message to the text display
    %   clearText          - clears the text display
    %   setLedColor        - sets the edge led color
    %   clearLedColor      - clears the edge led color
    %
    %   update             - updates internal state with the next feedback
    %   getFeedback        - gets 'io' and 'mobile' views of feedback
    %   getFeedbackIO      - gets 'io' view of feedback
    %   getFeedbackMobile  - gets 'mobile' view of feedback
    %   getOrientation     - gets 3x3 orientation matrix based on IMU data
    %   getArOrientation   - gets 3x3 orientation matrix based on AR data
    %   getArPosition      - gets 3x1 position vector based on AR data
    %   getArPose          - gets 4x4 6-dof pose based on AR data
    %
    %   sendVibrate        - sends a command to vibrate the device
    % 
    %   See also HebiLookup, HebiGroup
    
    %   Copyright 2014-2021 HEBI Robotics, Inc.
    
    properties(SetAccess = private)
        group HebiGroup;
    end
    
    properties
        retrySends logical = true;
    end
    
    properties(Access = private)
        ioFbk;
        mobileFbk;
        prevIoFbk;
        ioDiff struct = struct();
        lastSuccessTime;
    end
    
    % Static API
    methods(Static)
        
        function mobileIO = findDevice(familyName, deviceName)
            % searchController repeatedly searches the network until the
            % device is found
            timeout = 2;
            while true
                try
                    group = HebiLookup.newGroupFromNames( familyName, deviceName );
                    break;
                catch
                    % If we failed to make a group, pause a bit before trying again.
                    disp(['Timed out searching for mobileIO device: '...
                        '"' familyName '" | "' deviceName  '". Retrying...']);
                    pause(timeout - 1); % Java method timeout takes 1s
                end
            end
            mobileIO = HebiMobileIO(group);
        end
        
    end
    
    % Public API
    methods
        
        function this = HebiMobileIO(group)
            % HebiMobileIO Construct an instance of this class
            %
            %   The group must contain a single mobileIO device
            if group.getNumModules ~= 1
                error('The mobileIO group must contain a single device');
            end
            this.group = group;
            this.ioFbk = group.getNextFeedbackIO();
            this.prevIoFbk = group.get('feedback', 'view', 'io');
            this.mobileFbk = group.get('feedback', 'view', 'mobile');
            this.lastSuccessTime = tic();
        end
        
        function [] = initializeUI(this)
            % initializeUI initializes all UI elements to their default state
            ALL = 1:8;
            this.setAxisSnap(ALL, [0 0 nan nan nan nan 0 0]);
            this.setAxisValue(ALL, 0);
            this.setButtonToggle(ALL, false);
            this.setButtonIndicator(ALL, false);
            this.clearLedColor();
            this.clearText();
        end
        
        function [] = setAxisSnap(this, axes, values)
            % setAxisSnap sets the axis snap position
            %
            %   This method sets the 'snap' position of specified axes. When
            %   the snap value is set, an axis will automatically return  
            %   to once it is not actively being pressed. 'nan' deactivates
            %   snapping.
            %
            %   'Axes' Argument (required)
            %      The axis or an array of axes, e.g., [1 2] for a1 and a2
            %
            %   'Values' Argument (optional)
            %      A scalar, or an array of values specific for each axis.
            %      Defaults to zero when unspecified.
            %
            %   Example
            %     % Enable snapping on a1/a2 and disable on a3/a4
            %     mobileIO.setAxisSnap([1 2 3 4], [0 0 nan nan]);
            if nargin < 3
                values = 0;
            end
            this.setPins('a', axes, values);
        end
        
        function [] = setAxisValue(this, axes, values)
            % setAxisValue sets the axis position
            %
            %   This method sets the value of specified axes. This only
            %   works for non-snapping axes.
            %
            %   'Axes' Argument (required)
            %      The axis or an array of axes, e.g., [1 2] for a1 and a2
            %
            %   'Values' Argument (required)
            %      A scalar, or an array of values specific for each axis.
            %
            %   Example
            %     % Set a3/a4 to the lowest position
            %     mobileIO.setAxisValue([3 4], -1);
            this.setPins('f', axes, values);
        end
        
        function [] = setButtonToggle(this, buttons, values)
            % setButtonToggle sets the button toggle mode
            %
            %   'Buttons' Argument (required)
            %      The button or an array of buttons, e.g., [1 2] for b1 and b2
            %
            %   'Values' Argument (required)
            %      A scalar, or an array of values specific for each button.
            %      'true' (toggle mode): button remains on finger-up
            %      'false' (momentary mode): button disables on finger-up
            %
            %   Example
            %     % Set b1 to normal and b3 to toggle mode
            %     mobileIO.setButtonToggle([1 3], [false true]);
            this.setPins('b', buttons, logical(values));
        end
        
        function [] = setButtonIndicator(this, buttons, values)
            % setButtonIndicator sets a visual indicator around a button
            %
            %   'Buttons' Argument (required)
            %      The button or an array of buttons, e.g., [1 2] for b1 and b2
            %
            %   'Values' Argument (optional)
            %      A scalar, or an array of values specific for each button.
            %      'true': shows a visual indicator (default)
            %      'false': does not show a visual indicator
            %
            %   Example
            %     % Show an indicator on the active b1 and b8 buttons
            %     mobileIO.setButtonToggle([1 8], true);
            if nargin < 3
                values = true;
            end
            this.setPins('e', buttons, logical(values));
        end
        
        function [] = addText(this, text, clearPrevious)
            % addText appends a message to the text display
            if nargin < 3
                clearPrevious = false;
            end
            this.send('AppendLog', text, 'ClearLog', clearPrevious);
        end
        
        function [] = clearText(this)
            % clearText clears the text display
            this.send('ClearLog', true);
        end
        
        function [] = setLedColor(this, color)
            % setLedColor sets the edge led color
            %
            %   'Color' Argument (required)
            %      Can be a string argument ('red', 'green', 'magenta'),
            %      a numerical color ([r g b], [0.5 1 0]),
            %      or disabled/empty ([]).
            this.send('led', color);
        end
        
        function [] = clearLedColor(this)
            % clearLedColor clears the edge led color
            %
            % See also setLedColor
            this.setLedColor([]);
        end

        function [hasNewFeedback, feedbackAge] = update(this, varargin)
            % update updates internal state with the next feedback
            %
            %   This method is a wrapper around group.getNextFeedback()
            %   that updates multiple feedback structs at once. 
            %
            %   This method is blocking by default, but non-blocking
            %   behavior can be specified using the 'timeout' parameter.
            %   Non-blocking behavior is often useful when the mobile
            %   device is on wireless and may lose packets every once in a
            %   while. In many cases it is better to keep running with old
            %   data instead of blocking the entire demo.
            %   
            %   Example
            %      [hasNewFeedback, feedbackAge] = mobileIO.update('timeout',0);
            %      if feedbackAge > 10
            %        error('mobileIO has been stage for 10 seconds!');
            %      end
            %      if hasNewFeedback
            %        ikTarget = mobileIO.getArPose();
            %      end
            %
            %   See also HebiGroup.getNextFeedback
            tmpIoFbk = this.prevIoFbk;
            hasNewFeedback = ~isempty(this.group.getNextFeedback( ...
                tmpIoFbk, this.mobileFbk, varargin{:}));
            if hasNewFeedback
                this.lastSuccessTime = tic();
                feedbackAge = 0;
                this.prevIoFbk = this.ioFbk;
                this.ioFbk = tmpIoFbk;
            else
                feedbackAge = toc(this.lastSuccessTime);
            end
        end
        
        function [mobileFbk, ioFbk, ioFbkDiff] = getFeedback(this)
            % getFeedback returns 'mobile' and 'io' views of the latest feedback
            mobileFbk = this.mobileFbk;
            if nargout < 3
                [ioFbk] = this.getFeedbackIO();
            else
                [ioFbk, ioFbkDiff] = this.getFeedbackIO();
            end
        end
        
        function [ioFbk, ioFbkDiff] = getFeedbackIO(this)
            % getFeedbackIO returns the 'io' view of the latest feedback
            ioFbk = this.ioFbk;
            if nargout > 1
                ioFbkDiff = this.ioDiff;
                ioFbkDiff.a1 = this.ioFbk.a1 - this.prevIoFbk.a1;
                ioFbkDiff.a2 = this.ioFbk.a2 - this.prevIoFbk.a2;
                ioFbkDiff.a3 = this.ioFbk.a3 - this.prevIoFbk.a3;
                ioFbkDiff.a4 = this.ioFbk.a4 - this.prevIoFbk.a4;
                ioFbkDiff.a5 = this.ioFbk.a5 - this.prevIoFbk.a5;
                ioFbkDiff.a6 = this.ioFbk.a6 - this.prevIoFbk.a6;
                ioFbkDiff.a7 = this.ioFbk.a7 - this.prevIoFbk.a7;
                ioFbkDiff.a8 = this.ioFbk.a8 - this.prevIoFbk.a8;
                ioFbkDiff.b1 = this.ioFbk.b1 - this.prevIoFbk.b1;
                ioFbkDiff.b2 = this.ioFbk.b2 - this.prevIoFbk.b2;
                ioFbkDiff.b3 = this.ioFbk.b3 - this.prevIoFbk.b3;
                ioFbkDiff.b4 = this.ioFbk.b4 - this.prevIoFbk.b4;
                ioFbkDiff.b5 = this.ioFbk.b5 - this.prevIoFbk.b5;
                ioFbkDiff.b6 = this.ioFbk.b6 - this.prevIoFbk.b6;
                ioFbkDiff.b7 = this.ioFbk.b7 - this.prevIoFbk.b7;
                ioFbkDiff.b8 = this.ioFbk.b8 - this.prevIoFbk.b8;
            end
        end
        
        function [mobileFbk] = getFeedbackMobile(this)
            % getFeedbackMobile returns the 'mobile' view of the latest feedback
            mobileFbk = this.mobileFbk;
        end
        
        function [rotMat] = getOrientation(this)
            % getOrientation returns a 3x3 orientation matrix based on IMU data
            q = [
                this.mobileFbk.orientationW ...
                this.mobileFbk.orientationX ...
                this.mobileFbk.orientationY ...
                this.mobileFbk.orientationZ ];
            rotMat = HebiUtils.quat2rotMat(q);
        end
        
        function [rotMat, arQuality] = getArOrientation(this)
            % getArOrientation returns a 3x3 orientation matrix based on AR data
            q = [
                this.mobileFbk.arOrientationW ...
                this.mobileFbk.arOrientationX ...
                this.mobileFbk.arOrientationY ...
                this.mobileFbk.arOrientationZ ];
            rotMat = HebiUtils.quat2rotMat(q);
            if nargout > 1
                arQuality = this.mobileFbk.arQuality;
            end
        end
        
        function [translation, arQuality] = getArPosition(this)
            % getArPosition returns a 3x1 position vector based on AR data
            translation = [
                this.mobileFbk.arPositionX; ...
                this.mobileFbk.arPositionY; ...
                this.mobileFbk.arPositionZ ];
            if nargout > 1
                arQuality = this.mobileFbk.arQuality;
            end
        end
        
        function [pose, arQuality] = getArPose(this)
            % getArPose returns a 4x4 transform of the 6-dof pose based on AR data
            pose = eye(4);
            pose(1:3,1:3) = this.getArOrientation();
            pose(1:3,4) = this.getArPosition();
            if nargout > 1
                arQuality = this.mobileFbk.arQuality;
            end
        end
        
        function [] = sendVibrate(this, effort)
            % sendVibrate sends a command to vibrate the device
            %
            %   Note that this feature depends on device support. If the
            %   device does not support programmatic vibrating, then this
            %   will be a no-op.
            if nargin < 2
                effort = 1;
            end
            cmd = CommandStruct();
            cmd.effort = effort;
            this.group.send(cmd);
        end
        
    end
    
    % Internal utility methods
    methods(Access = private, Hidden)
        
        function [] = setPins(this, letter, indices, values)
            indices = indices(:);
            values = values(:);
            if isscalar(values)
                values = ones(size(indices)) * values;
            elseif length(values) ~= length(indices)
                error('length of values must be scalar or match the number of pins');
            end
            cmd = IoCommandStruct();
            for i = 1:length(indices)
                cmd.([letter num2str(indices(i))]) = values(i);
            end
            this.send(cmd);
        end
        
        function [] = send(this, varargin)
            if this.retrySends
                HebiUtils.sendWithRetry(this.group, varargin{:});
            else
                this.group.send(varargin{:});
            end
        end
        
    end
    
    % Inherited methods that we don't want to see in auto-complete
    % or generated documentation.
    methods(Access = public, Hidden = true)
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
    
end

