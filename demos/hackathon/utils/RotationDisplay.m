classdef RotationDisplay < handle
    %RotationDisplay displays the axes of a 3x3 rotation matrix
    %  x - red
    %  y - green
    %  z - blue
    %
    %  Example:
    %      % Slowly rotate about z axis
    %      rot = RotationDisplay();
    %      steps = 100;
    %      for angle = linspace(0, 2*pi, steps);
    %          c = cos(angle); s = sin(angle);
    %          rot.R = [
    %                1   0   0
    %                0  +c  -s
    %                0  +s  +c
    %          ];
    %          drawnow;
    %          pause(5/steps);
    %      end

    properties(Access = public)
        R
    end
    
    properties(Access = private)
        figHandle
        x
        y
        z
    end
    
    methods
        
        function this = RotationDisplay()
            this.figHandle = figure();
            hold on;
            
            % instantiate axes
            this.x = line( [0 1], [0 0], [0 0],'Color', 'r', 'LineWidth', 3);
            this.y = line( [0 0], [0 1], [0 0],'Color', 'g', 'LineWidth', 3);
            this.z = line( [0 0], [0 0], [0 1],'Color', 'b', 'LineWidth', 3);
            this.R = eye(3);
            
            % labels
            legend x y z
            title('Rotated Axes');
            xlabel('x');
            ylabel('y');
            zlabel('z');
            
            % set reasonable limits
            limit = [-1 1];
            xlim(limit);
            ylim(limit);
            zlim(limit);
            
            % draw additional lines through the center
            range = limit;
            line( range,[0 0], [0 0],'Color', 'k', 'LineStyle', '--');
            line( [0 0],range, [0 0],'Color', 'k', 'LineStyle', '--');
            line( [0 0],[0 0], range,'Color', 'k', 'LineStyle', '--');
            grid minor;
            box on;
            axis equal;
            hold off;
            view(3);
        end
        
        function set.R(this, T)
            sz = size(T);
            if ~isequal(sz, [3 3]) && ~isequal(sz, [4 4])
                error('expected 3x3 rotation matrix or 4x4 transform');
            end
            this.R = T(1:3,1:3);
            set( this.x, 'XData', [0 T(1,1)], 'YData', [0 T(2,1)], 'ZData', [0 T(3,1)] );
            set( this.y, 'XData', [0 T(1,2)], 'YData', [0 T(2,2)], 'ZData', [0 T(3,2)] );
            set( this.z, 'XData', [0 T(1,3)], 'YData', [0 T(2,3)], 'ZData', [0 T(3,3)] );
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

