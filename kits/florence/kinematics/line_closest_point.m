function [ closest ] = line_closest_point( point, a, n )
%LINE_CLOSEST_POINT takes in a point and two vectors describing a line. 'a'
%is the origin offset, and 'n' is the unit vector.  It returns closest
%point on the line to the point given.

    % Check the inputs to make sure vectors are the right dimensions, so 
    % that the function will work with any mix of row or column vectors.  
    % This function will return a column vector.
    if size(point,1) < size(point,2)
        point = point';
    end
    
    if size(a,1) < size(a,2)
        a = a';
    end
    
    if size(n,1) < size(n,2)
        n = n';
    end
    
    % The maths are from wikipedia:
    % http://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line 
    closest = a + dot((point - a),n)*n;

end

