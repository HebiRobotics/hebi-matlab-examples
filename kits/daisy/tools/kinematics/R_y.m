function DCM = R_y( angle )
% SO(3) Rotation matrix about y-axis by an angle in radians

DCM = [ cos(angle)  0  sin(angle);
        	0       1       0;
       -sin(angle)  0  cos(angle) ];

end
