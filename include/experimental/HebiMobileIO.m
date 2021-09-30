classdef HebiMobileIO < handle
    %HebiMobileIO is utility wrapper for the mobileIO phone app
    %   
    %   HebiMobileIO Methods:
    %
    %   setDefaults          - sets all axes and buttons to their default configuration
    %   setAxisSnap          - configures snapping behavior desired axes
    %   setAxisValue         - sets the current axis value
    %   setButtonToggle      - configures button toggle mode
    %   setButtonIndicator   - configures visual button indicators
    %   setColor             - sets the color of a visual indicator on the display
    
    %   Copyright 2014-2021 HEBI Robotics, Inc.
    
    properties
        sendWithRetry = true;
    end
    
    properties(SetAccess = private)
        group HebiGroup;
    end
    
    properties(Access = private)
        ioFbk;
        mobileFbk;
        prevIoFbk;
        ioDiff struct = struct();
        successTime;
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
            this.successTime = tic();
        end
        
        function [] = setDefaults(this)
            % setDefaults sets all axes and buttons to their default configuration
            ALL = 1:8;
            this.setAxisSnap(ALL, [0 0 nan nan nan nan 0 0]);
            this.setAxisValue(ALL, 0);
            this.setButtonToggle(ALL, false);
            this.setButtonIndicator(ALL, false);
            this.setColor([]);
            this.clearText();
        end
        
        function [] = setAxisSnap(this, axes, values)
            % setAxisSnap configures snapping behavior for axes
            %
            %   This method sets the 'snap' value of specified axes. When
            %   the snap value is set, an axis will automatically return  
            %   to the snap-value once it is not actively being pressed. 
            %   'nan' deactivates snapping.
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
            % setAxisValue sets the current axis value
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
            % setButtonToggle configures button toggle mode
            %
            %   'Buttons' Argument (required)
            %      The button or an array of buttons, e.g., [1 2] for b1 and b2
            %
            %   'Values' Argument (required)
            %      A scalar, or an array of values specific for each button.
            %      'true' (toggle mode): button remains on finger-up
            %      'false' (normal mode): button disables on finger-up
            %
            %   Example
            %     % Set b1 to normal and b3 to toggle mode
            %     mobileIO.setButtonToggle([1 3], [false true]);
            this.setPins('b', buttons, logical(values));
        end
        
        function [] = setButtonIndicator(this, buttons, values)
            % setButtonIndicator configures visual button indicators
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
        
        function [] = setColor(this, color)
            % setColor sets the color of a visual indicator on the display
            %
            %   'Color' Argument (required)
            %      Can be a string argument ('red', 'green', 'magenta'), 
            %      a numerical color ([r g b], [0.5 1 0]), 
            %      or disabled/empty ([]).
            this.send('led', color);
        end
        
        function [] = clearText(this)
            this.send('ClearLog', true);
        end
        
        function [] = sendText(this, text, clearPrevious)
            if nargin < 3
                clearPrevious = false;
            end
            this.send('AppendLog', text, 'ClearLog', clearPrevious);
        end
        
        function [hasUpdated, secondsSinceSuccess] = update(this, varargin)
            % TODO: fix comment (copy & paste from example)
            %
            % We get feedback from the phone into the existing structs. The
            % timeout of 0 means that the method returns immediately and won't
            % wait for feedback. If there was no feedback, the method returns
            % empty ([]), and the data in the passed in structs does not get
            % overwritten.
            % We do this because the mobile device is typically on wireless and
            % might drop out or be really delayed, in which case we would
            % rather keep running with an old data instead of waiting here for
            % new data.
            tmpIoFbk = this.prevIoFbk;
            hasUpdated = ~isempty(this.group.getNextFeedback( ...
                tmpIoFbk, this.mobileFbk, varargin{:}));
            if hasUpdated
                this.successTime = tic();
                secondsSinceSuccess = 0;
                this.prevIoFbk = this.ioFbk;
                this.ioFbk = tmpIoFbk;
            else
                secondsSinceSuccess = toc(this.successTime);
            end
        end
        
        function [ioFbk, ioFbkDiff] = getFeedbackIO(this)
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
            mobileFbk = this.mobileFbk;
        end
        
        function [rotMat] = getOrientation(this)
            q = [
                this.mobileFbk.orientationW ...
                this.mobileFbk.orientationX ...
                this.mobileFbk.orientationY ...
                this.mobileFbk.orientationZ ];
            rotMat = HebiUtils.quat2rotMat(q);
        end
        
        function [rotMat, arQuality] = getArOrientation(this)
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
            translation = [
                this.mobileFbk.arPositionX; ...
                this.mobileFbk.arPositionY; ...
                this.mobileFbk.arPositionZ ];
            if nargout > 1
                arQuality = this.mobileFbk.arQuality;
            end
        end
        
        function [pose, arQuality] = getArPose(this)
            pose = eye(4);
            pose(1:3,1:3) = this.getArOrientation();
            pose(1:3,4) = this.getArPosition();
            if nargout > 1
                arQuality = this.mobileFbk.arQuality;
            end
        end
        
    end
    
    % Internal utility methods
    methods(Access = private)
        
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
            if this.sendWithRetry
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

