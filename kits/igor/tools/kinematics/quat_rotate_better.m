function [ q_out ] = quat_rotate_better( w_x, w_y, w_z, q_bottom, dt )
%QUAT_ROTATE Rotate a quaternion by a velocity and timestep.
%
%   Details of the maths are from:
%   Sigma-Point Kalman Filters for Nonlinear Estimation and Sensor-Fusion
%   van der Merwe, Wan and Julier
    
    % Switch to scalar top, because that's what this code was written for
    q = zeros(4,1);
    q(1) =  q_bottom(4);   % w  -x
    q(2) = -q_bottom(1);   % x  -y  
    q(3) = -q_bottom(2);   % y  -z  
    q(4) = -q_bottom(3);   % z   w
    
%     q(1) = q_bottom(4);  % w 
%     q(2) = q_bottom(1);  % x   
%     q(3) = q_bottom(2);  % y 
%     q(4) = q_bottom(3);  % z

    
    % Make the velocities into differential rotations
    d_x = w_x * dt;
    d_y = w_y * dt;
    d_z = w_z * dt;  

    % Construct 4 x 4 skew symmetric matrix
    Q_vel = [  0    d_x   d_y   d_z;
             -d_x    0   -d_z   d_y;
             -d_y   d_z    0   -d_x;
             -d_z  -d_y   d_x    0 ];
         
    % Norm of velocity vector for doing a cleaner quaternion update
    s = .5 * norm( [ d_x,  d_y,  d_z ] );
    
    if s==0
        s = 1E-6;
    end
    
    % Update using some quaternion calculus.
    q_new = (eye(4) * cos(s) - .5 * Q_vel * sin(s) / s) * q;
    
%     % Normalize
%     q_new = q_new / ...
%        (q_new(1)^2 + q_new(2)^2 + q_new(3)^2 + q_new(4)^2);
   
   
    % Switch back to using scalar bottom to spit out
    q_out = zeros(4,1);
    q_out(1) = -q_new(2);    % x  w
    q_out(2) = -q_new(3);    % y -x
    q_out(3) = -q_new(4);    % z -y
    q_out(4) =  q_new(1);    % w -z
    
%     q_out(1) = q_new(2);  % x 
%     q_out(2) = q_new(3);  % y  
%     q_out(3) = q_new(4);  % z  
%     q_out(4) = q_new(1);  % w   

end

