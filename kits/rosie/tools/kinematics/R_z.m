function DCM = R_z( angle )
% SO(3) Rotation matrix about z-axis by an angle in radians

DCM = [ cos(angle) -sin(angle)  0;
        sin(angle)  cos(angle)  0;
            0           0       1 ];

end
