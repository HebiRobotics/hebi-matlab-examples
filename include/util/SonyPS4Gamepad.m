classdef (Sealed) SonyPS4Gamepad < handle
    % SonyPS4Gamepad wraps a joystick object in a way that is platform
    % independent and easier to access. Assumes using a Sony PS4 Gamepad:
    %
    %   Model CUH-ZCT2U Wireless Controller
    
    %   Copyright 2014-2018 HEBI Robotics, Inc.
    
    % Visible properties
    properties(SetAccess = private)
        joy
    end
    
    % Internal state
    properties(SetAccess = private, Hidden = true)
        lastAxes {isnumeric};
        lastButtons {isnumeric};
        lastPovs {isnumeric};
    end
    
    properties(SetAccess = public)
        axisLowpass {isnumeric} = 1; % [0-1] where 1 is no lowpass
        axisDeadZone {isnumeric} = 0.1;
        axisDeadZonePostLowpass {isnumeric} = 0.06;
    end
    
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
        
        function this = SonyPS4Gamepad(joy)
            % constructor. Takes a joystick object
            if ~isa(joy, 'vrjoystick') && ~isa(joy, 'HebiJoystick')
                error('Expected vrjoystick or HebiJoystick');
            end
            this.joy = joy;
            [this.lastAxes, this.lastButtons, this.lastPovs] = read(joy);
            
        end
        
        function result = hasInitialConditions(this)
            % Checks that joystick has expected initial conditions. This
            % exists to work around a bug that makes the triggers not work
            % correctly until they are both pushed down fully for the first
            % time. Also checks for a bug where all of the axes are
            % initialized to -1 in Linux.
            
            % Read current state (without lowpass)
            [this.lastAxes, this.lastButtons, this.lastPovs] = read(this.joy);
            deadZone = abs(this.lastAxes) < this.axisDeadZone;
            this.lastAxes(deadZone) = 0;
            state = SonyPS4Gamepad.toState(this.lastAxes, this.lastButtons, this.lastPovs);

            % Issue depends on the OS
            if isunix % includes mac
                result = ...
                    state.AXIS_LEFT_TRIGGER == -1  && ...
                    state.AXIS_RIGHT_TRIGGER == -1 && ...
                    state.AXIS_RIGHT_STICK_X == 0 && ...
                    state.AXIS_RIGHT_STICK_Y == 0 && ...
                    state.AXIS_LEFT_STICK_X == 0 && ...
                    state.AXIS_LEFT_STICK_Y == 0 && ...
                    state.BUTTON_RIGHT_TRIGGER == 0 && ...
                    state.BUTTON_LEFT_TRIGGER == 0;
            else
                % TODO: Copied from previous version. Is this correct?
                % My Joystick axes map to [0-1] on Windows, so they can
                % never get to -1. Also, should we allow for a small delta
                % in case the joystick isn't calibrated perfectly?
                % /fenner
                result = ...
                    state.AXIS_LEFT_TRIGGER == -1  && ...
                    state.AXIS_RIGHT_TRIGGER == -1;
            end
            
        end
        
        function [state, delta] = read(this)
            
            % Read joystick
            [axes, buttons, povs] = read(this.joy);
            
            % Ignore values within deadzone
            deadZone = abs(axes) < this.axisDeadZone;
            axes(deadZone) = 0;
            
            % Lowpass axes
            axes = this.axisLowpass * axes + ...
                (1-this.axisLowpass) * this.lastAxes;
            
            % Do another deadzone check after lowpassing to avoid the
            % lowpassed result to never return to zero.
            deadZone = abs(axes) < this.axisDeadZonePostLowpass;
            axes(deadZone) = 0;
            
            % Convert to structs
            state = SonyPS4Gamepad.toState(axes,buttons,povs);
            delta = SonyPS4Gamepad.toState(...
                axes - this.lastAxes, ...
                buttons - this.lastButtons, ...
                povs - this.lastPovs);
            
            % Store current values
            this.lastAxes = axes;
            this.lastButtons = buttons;
            this.lastPovs = povs;
            
        end

    end
    
     methods(Static, Hidden = true, Access = public)
          function state = toState(axes, buttons, povs)
            % Converts state vectors to named struct
            
            state = struct();
            state.AXIS_LEFT_STICK_X = axes(1);
            state.AXIS_LEFT_STICK_Y = axes(2);
            state.BUTTON_OPTIONS = buttons(10);
            state.BUTTON_LEFT_TRIGGER = buttons(5);
            state.BUTTON_RIGHT_TRIGGER = buttons(6);
            state.BUTTON_SHARE = buttons(9);
            state.POV = povs(1);

            % NOTE: some axes/buttons are marked because my test-joystick
            % doesn't have them. Should be uncommented
            
            if ispc || ismac
                state.AXIS_LEFT_TRIGGER = axes(4);
                state.AXIS_RIGHT_TRIGGER = axes(5);
                state.AXIS_RIGHT_STICK_X = axes(3);
                state.AXIS_RIGHT_STICK_Y = axes(6); %
                state.BUTTON_SQUARE = buttons(1);
                state.BUTTON_CIRCLE = buttons(3);
                state.BUTTON_TRIANGLE = buttons(4);
                state.BUTTON_X = buttons(2);
                state.BUTTON_LEFT_STICK_CLICK = buttons(11); %
                state.BUTTON_TOUCHPAD = buttons(14); %
            else
                state.AXIS_LEFT_TRIGGER = axes(3);
                state.AXIS_RIGHT_TRIGGER = axes(6);
                state.AXIS_RIGHT_STICK_X = axes(4);
                state.AXIS_RIGHT_STICK_Y = axes(5);
                state.BUTTON_SQUARE = buttons(4);
                state.BUTTON_CIRCLE = buttons(2);
                state.BUTTON_TRIANGLE = buttons(3);
                state.BUTTON_X = buttons(1);
                state.BUTTON_LEFT_STICK_CLICK = buttons(12);
                state.BUTTON_TOUCHPAD = buttons(11);
            end
            
          end
     end
     
    methods(Access = public, Hidden = true)
        function [] = delete(~)
            %destructor
        end
    end
    
    
end
