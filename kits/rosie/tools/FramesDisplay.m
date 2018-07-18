classdef FramesDisplay < handle
    %FramesDisplay draws the coordinate frames of each module in a
    %kinematic chain. The axes are color-coded RGB for XYZ.
    %
    %  FramesDisplay = FramesDisplay() opens a new figure for drawing
    %  coordinate frames for transforms. The figure gets closed
    %  automatically when this variable is deleted.
    %
    %  The frames get updated using FrameDisplay.setFrames(), where the
    %  input is a new set of 4x4xN homogeneous transforms.  In order to get
    %  the figure to re-render you will need to execute a DRAWNOW command
    %  or some other call the forces a re-draw of graphics objects, like a
    %  PAUSE.
    %
    %  'axisLength' optionally specifies the length of each axis in [m].
    %
    %  'numFrames' optionally specifies the number of coordinate frames and
    %  initializes the figure handles in the constructor. Otherwise the
    %  figure handles will be initialized at the first call to setFrames.
    %
    %  Axis colors:
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
    %     framesDisplay = FramesDisplay();
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
    %             % drawnow; % If you don't have pause somewhere in the
    %                        % loop you will need to do this to force the 
    %                        % figure to re-render.
    %         end
    %     end
    
    properties (Access = public)
        figHandle
        axHandle
        x
        y
        z
        numFrames
        axisLength = 0.1;
    end
    
    methods(Access = public)
        
        function this = FrameDisplay(axisLength, numFrames)
            % Constructor gets called once. It creates a new figure and
            % formats it nicely.
            if nargin > 0
                this.axisLength = axisLength; 
            end
            if nargin > 1
                this.init(numFrames);
            end
        end
        
        function init(this, numFrames)
            % init sets up the display that gets updated by draw
            this.numFrames = numFrames;
            
            this.figHandle = figure();
            this.axHandle = axes();
            hold on;
            
            % Initialize all axes to identity. Iterate backwards so that
            % we don't need to pre-initialize arrays
            range = [0 this.axisLength];
            for i = numFrames:-1:1
                this.x(i) = line(this.axHandle, range, [0 0], [0 0],'Color', 'r', 'LineWidth', 3);
                this.y(i) = line(this.axHandle, [0 0], range, [0 0],'Color', 'g', 'LineWidth', 3);
                this.z(i) = line(this.axHandle, [0 0], [0 0], range,'Color', 'b', 'LineWidth', 3);
            end
            
            % Draw small coordinate frame in the center
            line( range,[0 0], [0 0],'Color', 'k', 'LineWidth', 2);
            line( [0 0],range, [0 0],'Color', 'k', 'LineWidth', 2);
            line( [0 0],[0 0], range,'Color', 'k', 'LineWidth', 2);
            grid on;
            box on;
            axis equal;
            %axis auto;  
            set(gca, 'PlotBoxAspectRatio', [1 1 1]);
            hold off;
            view(3); 
            
            % labels
            legend x y z
            title('Frames');
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
            newFramesSize = size(frames);
            if length(newFramesSize) > 2
                newNumFrames = newFramesSize(3);
            else
                newNumFrames = 1;
            end
            
            if ~isequal(newFramesSize(1:2), [4 4])
                error('expected input: 4x4xN homogeneous transforms');  
            end
            
            if ~isempty(frames)
                if newNumFrames < this.numFrames
                    this.frames(:,:,newNumFrames+1:end) = [];
                end
                if newNumFrames > this.numFrames
                    range = [0 this.axisLength];
                    for i = this.numFrames+1:newNumFrames
                        this.x(i) = line(this.axHandle, range, [0 0], [0 0],'Color', 'r', 'LineWidth', 3);
                        this.y(i) = line(this.axHandle, [0 0], range, [0 0],'Color', 'g', 'LineWidth', 3);
                        this.z(i) = line(this.axHandle, [0 0], [0 0], range,'Color', 'b', 'LineWidth', 3);
                    end
                end
            end
            
            this.numFrames = newNumFrames;
            
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

