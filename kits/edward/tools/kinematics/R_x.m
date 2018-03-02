function DCM = R_x( angle )
% SO(3) Rotation matrix about x-axis by an angle in radians

DCM = [ 1       0           0;
        0   cos(angle)  -sin(angle);
        0   sin(angle)   cos(angle) ];
end
