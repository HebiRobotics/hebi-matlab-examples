function chassisPose = getBodyPoseFromFeet( feetXYZ, baseXYZ )
% GETBODYPOSEFROMFEET Calculates the translation and rotation of the
% chassis based on the XYZ locations of the feet from some "baseXYZ"
% position.
%
% INPUT:        feetXYZ = 3x1xN feedback position of feet
%               baseXYZ = 3x1xN 'home' position of feet
%
% Dave Rollinson
% Dec 2014

    numFeet = size(feetXYZ,2);
    
    % Zero mean the points
    feetXYZ_CoM = mean(feetXYZ,2);
    baseXYZ_CoM = mean(baseXYZ,2);
    
    feetXYZ = feetXYZ - repmat( feetXYZ_CoM, 1, numFeet );
    baseXYZ = baseXYZ - repmat( baseXYZ_CoM, 1, numFeet );
    
    % Build the weighted correlation matrix
    xyzCorr = baseXYZ * feetXYZ';
    
    % Take the SVD
    [U,~,V] = svd( xyzCorr );
    
    % Modify the scaling matrix to account for reflections
    S = eye(3);
    S(3,3) = sign(det(U*V));
    
    % Make the final transform
    chassisPose = eye(4);
    R = V*S*U';
    t = feetXYZ_CoM - baseXYZ_CoM;

    chassisPose(1:3,1:3) = R;
    chassisPose(1:3,4) = t;
end