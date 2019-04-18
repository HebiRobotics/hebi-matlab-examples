function chassisPose = getPoseFromXYZPoints( testXYZ, baseXYZ )
% GETPOSEFROMXYZPOINTS Calculates the translation and rotation of the
% chassis based on the XYZ locations of the feet from some "baseXYZ"
% position.
%
% INPUT:        testXYZ = 3xN points taken from test positions of an arm
%               baseXYZ = 3xN 'home' frame of points
%
% Dave Rollinson
% Dec 2014

    numPoints = size(testXYZ,2);
    
    % Zero mean the points
    testXYZ_centroid = mean(testXYZ,2);
    baseXYZ_centroid = mean(baseXYZ,2);
    
    testXYZ = testXYZ - repmat( testXYZ_centroid, 1, numPoints );
    baseXYZ = baseXYZ - repmat( baseXYZ_centroid, 1, numPoints );
    
    % Build the weighted correlation matrix
    xyzCorr = baseXYZ * testXYZ';
    
    % Take the SVD
    [U,~,V] = svd( xyzCorr );
    
    % Modify the scaling matrix to account for reflections
    S = eye(3);
    S(3,3) = sign(det(U*V));
    
    % Make the final transform
    chassisPose = eye(4);
    R = V*S*U';
    t = testXYZ_centroid - R*baseXYZ_centroid;

    chassisPose(1:3,1:3) = R;
    chassisPose(1:3,4) = t;
    
    
end