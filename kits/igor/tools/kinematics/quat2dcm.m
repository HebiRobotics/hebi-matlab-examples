function [ rot_mat ] = quat2dcm( q )
%QUAT2DCM Conversion of a quaternion to an orthogonal rotation matrix.
%Assumes that scalar (w) is the last element of the quaternion vector.
  
    rot_mat = zeros(3,3,size(q,2));
    
    %%%%%%%%%%%%%%%%%
    % For-Loop Code %
    %%%%%%%%%%%%%%%%%
    
    % Believe it or not this code is 2-3X faster than the vectorized code.
    for i=1:size(q,2)
     
        a = q(1,i);
        b = q(2,i);
        c = q(3,i);
        d = q(4,i);
        
        rot_mat(:,:,i) = [ a^2 - b^2 - c^2 + d^2,   2*(a*b + c*d),       2*(a*c - b*d);
                             2*(a*b - c*d),   -a^2 + b^2 - c^2 + d^2,    2*(b*c + a*d);
                             2*(a*c + b*d),         2*(b*c - a*d),  -a^2 - b^2 + c^2 + d^2];
    end
    
%     %%%%%%%%%%%%%%%%%%%
%     % Vectorized Code %
%     %%%%%%%%%%%%%%%%%%%
%     Q = reshape( q, 1, 4, size(q,2) );
%    
%     rot_mat=[Q(1,1,:).^2-Q(1,2,:).^2-Q(1,3,:).^2+Q(1,4,:).^2, 2*(Q(1,1,:).*Q(1,2,:)+Q(1,3,:).*Q(1,4,:)), 2*(Q(1,1,:).*Q(1,3,:)-Q(1,2,:).*Q(1,4,:));
%             2*(Q(1,1,:).*Q(1,2,:)-Q(1,3,:).*Q(1,4,:)), -Q(1,1,:).^2+Q(1,2,:).^2-Q(1,3,:).^2+Q(1,4,:).^2, 2*(Q(1,2,:).*Q(1,3,:)+Q(1,1,:).*Q(1,4,:));
%             2*(Q(1,1,:).*Q(1,3,:)+Q(1,2,:).*Q(1,4,:)), 2*(Q(1,2,:).*Q(1,3,:)-Q(1,1,:).*Q(1,4,:)), -Q(1,1,:).^2-Q(1,2,:).^2+Q(1,3,:).^2+Q(1,4,:).^2];
% 

end

