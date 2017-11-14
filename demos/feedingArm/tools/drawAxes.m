function [ animate_struct ] = drawAxes( animate_struct, ...
                                axis_frames, axis_lengths )
% DRAWAXES Draws the coordinate frames of each of the links in a robot.  
% The axes are color-coded RGB for XYZ.
%
% [ animate_struct ] = drawxes(animate_struct, 
%                                axis_frames, axis_lengths)
%
%   axis_frames
%       4x4xN set of transforms describing the pose of each frames in a 
%       common base body frame.  N is the number of frames.
%
%   axis_lengths (optional)
%       3x1 set of lengths (in inches) for drawing the axes.
%
%   animate_struct 
%       Struct with info about drawing the snake.  You can pass in a blank
%       struct the first time, by passing in struct() and the necessary
%       information will get filled in the first time thru.
%
%   animate_struct.fig
%       The handle to the figure that the snake gets drawn in.  If you
%       want, you can create a figure manually and save the handle here.
%
% Dave Rollinson
% 2010 - 2013 


%   THESE PROPERTIES BELOW GET SET INTERNALLY.  You probably don't need 
%   to worry about them.
%
%   animate_struct.x
%   animate_struct.y
%   animate_struct.z
%       Variables for the module coordinate frames.  Updated internally to
%       drawAxes

    % If a figure handle is not yet specified, make a new figure
    if ~isfield(animate_struct,'fig')
        animate_struct.fig = figure(42);
    elseif ~isfield(animate_struct,'x')
        set(0,'CurrentFigure',animate_struct.fig);
    end
    
    % Default Axis Lengths if not specified
    if nargin < 3
        axis_lengths = 0.1 * ones(3,1); % 10 cm axis length by default
    end

    % If there are no surface handles, plot everything from scratch
    if  ~isfield(animate_struct,'x')
        
        % Clear the figure to get ride of any old line objects
        % cla(animate_struct.ax)    
        for module = 1:size(axis_frames,3)   
            
            % Transform the cylinder to the module's position and
            % orientation.
            T = axis_frames(:,:,module);
              
            %%%%%%%%%%%%%%%
            % Module Axes %
            %%%%%%%%%%%%%%%

            % Transform principle axes to the position / orientation of
            % each module.
            axes_T = T * [diag(axis_lengths); ones(1,3)];
            orig_T = T * [zeros(3); ones(1,3)];
            animate_struct.x(module) = line( [orig_T(1,1) axes_T(1,1)],...
                                        [orig_T(2,1) axes_T(2,1)],...
                                        [orig_T(3,1) axes_T(3,1)],...
                                        'Color', 'r', 'LineWidth', 3);
            animate_struct.y(module) = line( [orig_T(1,2) axes_T(1,2)],...
                                        [orig_T(2,2) axes_T(2,2)],...
                                        [orig_T(3,2) axes_T(3,2)],...
                                        'Color', 'g', 'LineWidth', 3);
            animate_struct.z(module) = line( [orig_T(1,3) axes_T(1,3)],...
                                        [orig_T(2,3) axes_T(2,3)],...
                                        [orig_T(3,3) axes_T(3,3)],...
                                        'Color', 'b', 'LineWidth', 3);
            
            if module==1
                hold on;
            end
        end
        
        hold off;
        animate_struct.ax = gca;
        axis equal;
        grid on;
        
    else

        % If we've already drawn once use 'set' so things render much
        % faster (helps if trying to visualize things online).
        for module = 1:size(axis_frames,3)
            
            % Transform the cylinder to the module's position and
            % orientation.
            T = axis_frames(:,:,module);

            %%%%%%%%%%%%%%%
            % Module Axes %
            %%%%%%%%%%%%%%%

            % Transform principle axes to the position / orientation of
            % each module.
            axes_T = T * [diag(axis_lengths); ones(1,3)];
            orig_T = T * [zeros(3); ones(1,3)];
            set( animate_struct.x(module), ...
                                'XData', [orig_T(1,1) axes_T(1,1)],...
                                'YData', [orig_T(2,1) axes_T(2,1)],...
                                'ZData', [orig_T(3,1) axes_T(3,1)] );
            set( animate_struct.y(module), ...
                                'XData', [orig_T(1,2) axes_T(1,2)],...
                                'YData', [orig_T(2,2) axes_T(2,2)],...
                                'ZData', [orig_T(3,2) axes_T(3,2)] );
            set( animate_struct.z(module), ...
                                'XData', [orig_T(1,3) axes_T(1,3)],...
                                'YData', [orig_T(2,3) axes_T(2,3)],...
                                'ZData', [orig_T(3,3) axes_T(3,3)] );
        end    
    end

end
