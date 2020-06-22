classdef FrameDisplay < handle
    %FrameDisplay draws the coordinate frames of each module in a
    %kinematic chain. The axes are color-coded RGB for XYZ.
    %
    %  FrameDisplay = FrameDisplay() opens a new figure for drawing
    %  coordinate frames for transforms. The figure gets closed
    %  automatically when this variable is deleted.
    %
    %  Once set up, you display a coordinate frame with .setFrames(frames),
    %  where frames is a 4x4xN set of N homogeneous transforms.  See the
    %  example below.
    %
    %  Optional Initialization Arguments:
    %  FrameDisplay = FrameDisplay( AXISLENGTH, NUMFRAMES, XYZLIMITS )
    %
    %  AXISLENGTH optionally specifies the length of each axis in [m], if
    %  using later arguments without setting AXISLENGTH, use empty [].
    %  Default length is 0.05 [m].
    %
    %  NUMFRAMES optionally specifies the number of coordinate frames and
    %  initializes the figure handles in the constructor. Otherwise the
    %  figure handles will be initialized at the first call to setFrames.
    %  If using later arguments without setting NUMFRAMES, use empty [].
    %
    %  XYZLIMITS optionally specifies and fixes the limits of the plot.  If 
    %  it is not set the plot will scale automatically with each redraw.
    %
    %  Frame Axis Color Coding:
    %  x - red
    %  y - green
    %  z - blue
    %
    %  Example:
    %     % Initialize kinematic structure
    %     kin = HebiKinematics();
    %     kin.addBody('X5-4');
    %     kin.addBody('X5Link', 'ext', 0.350, 'twist', 0); 
    %     kin.addBody('X5-1');
    % 
    %     % Create display
    %     framesDisplay = FrameDisplay();
    % 
    %     % Move all joints individually in a circle and
    %     % continuously update display
    %     numDof = kin.getNumDoF();
    %     steps = 50;
    %     delay = 10 / (steps * numDof);
    %     for i = numDof:-1:1
    %     positions = zeros(1,numDof);
    %         for angle = linspace(0, 2*pi, steps)
    %             positions(i) = angle;
    %             frames = kin.getFK('output', positions);
    %             framesDisplay.setFrames(frames); % update visualization
    %             pause(delay);
    %         end
    %     end
    
    properties (Access = private)
        figHandle
        axHandle
        x
        y
        z
        numFrames
        axisLength = 0.05;  % m 
        xyzLimits = nan(3,2); % m (stacked [xLim; yLim; zLim])
        fixedLimits = false;
    end
    
    methods(Access = public)
        

        function this = FrameDisplay(axisLength, numFrames, xyzLimits)

            % Constructor gets called once. It creates a new figure and
            % formats it nicely.
            if nargin > 0 && ~isempty(axisLength)
                this.axisLength = axisLength; 
            end

            if nargin > 1 && ~isempty(numFrames)
                this.init(numFrames);
            end
            if nargin > 2 && ~isempty(xyzLimits)
                this.xyzLimits = xyzLimits;
                this.fixedLimits = true;
            end

        end
        
        function init(this, numFrames)
            % init sets up the display that gets updated by draw
            this.numFrames = numFrames;
            
            this.figHandle = figure();
            hold on;
            
            % Initialize all axes to identity. Iterate backwards so that
            % we don't need to pre-initialize arrays
            range = [0 this.axisLength];
            for i = numFrames:-1:1
                this.x(i) = line( range, [0 0], [0 0],'Color', 'r', 'LineWidth', 3);
                this.y(i) = line( [0 0], range, [0 0],'Color', 'g', 'LineWidth', 3);
                this.z(i) = line( [0 0], [0 0], range,'Color', 'b', 'LineWidth', 3);
            end
            
            % Draw black coordinate frame at the origin
            line( range,[0 0], [0 0],'Color', 'k', 'LineWidth', 2);
            line( [0 0],range, [0 0],'Color', 'k', 'LineWidth', 2);
            line( [0 0],[0 0], range,'Color', 'k', 'LineWidth', 2);
            grid on;
            axis equal;
            
            this.axHandle = gca;
            
            set(this.axHandle, 'PlotBoxAspectRatio', [1 1 1]);

            hold off;
            view(3); 
            
            % labels
            legend x y z
            title('Robot Coordinate Frames');
            xlabel('x (m)');
            ylabel('y (m)');
            zlabel('z (m)');

        end
        
        function setFrames(this, frames)
            
            % Initialize if it hasn't been done yet
            if isempty(this.figHandle)
                this.init(size(frames,3));
            end
            
            % Draw gets called often and continuously updates the frames
            if this.numFrames == 1 
                if ~isequal(size(frames), [4 4])
                  error('expected input: 4x4');  
                end
            else
                if ~isequal(size(frames), [4 4 this.numFrames])
                    error(['expected input: 4x4x' num2str(this.numFrames)]);
                end
            end
            
            xlim(this.axHandle,'auto');
            ylim(this.axHandle,'auto');
            zlim(this.axHandle,'auto');
            
            axes_base = [eye(3) * this.axisLength; ones(1,3)];
            orig_base = [zeros(3); ones(1,3)];
            for i = 1:this.numFrames
                % Transform principle axes to the position / orientation of
                % each module.
                T = frames(:,:,i);
                axes_T = T * axes_base;
                orig_T = T * orig_base;
                
                % Update existing line handles
                set( this.x(i), ...
                    'XData', [orig_T(1,1) axes_T(1,1)], ...
                    'YData', [orig_T(2,1) axes_T(2,1)], ...
                    'ZData', [orig_T(3,1) axes_T(3,1)] );
                set( this.y(i), ...
                    'XData', [orig_T(1,2) axes_T(1,2)], ...
                    'YData', [orig_T(2,2) axes_T(2,2)], ...
                    'ZData', [orig_T(3,2) axes_T(3,2)] );
                set( this.z(i), ...
                    'XData', [orig_T(1,3) axes_T(1,3)], ...
                    'YData', [orig_T(2,3) axes_T(2,3)], ...
                    'ZData', [orig_T(3,3) axes_T(3,3)] );
            end
            
            % If we didn't set the axis limits, let them auto-update if
            % they grow larger, never shrink them
            if ~this.fixedLimits
                xyzLimitsNew(1,:) = get(this.axHandle,'XLim');
                xyzLimitsNew(2,:) = get(this.axHandle,'YLim');
                xyzLimitsNew(3,:) = get(this.axHandle,'ZLim');

                this.xyzLimits(:,1) = min( [this.xyzLimits(:,1) xyzLimitsNew(:,1)], [], 2 );
                this.xyzLimits(:,2) = max( [this.xyzLimits(:,2) xyzLimitsNew(:,2)], [], 2 );      
            end
            
            xlim(this.axHandle,this.xyzLimits(1,:));
            ylim(this.axHandle,this.xyzLimits(2,:));
            zlim(this.axHandle,this.xyzLimits(3,:));
            
        end
        
    end
    
    methods (Access = private)
        function delete(this)
            % close figure if it is still open
            if any(findall(0,'Type','Figure') == this.figHandle)
                close(this.figHandle);
            end
        end
    end
    
end

