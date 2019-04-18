function [ footWrenches, distVec ] = getFootStanceWrenches( ...
                   gravityVec, stanceXYZ, isFlight, robotMass, robotCoM )
%GETFOOTSTANCECOEFFS Summary of this function goes here
%   Detailed explanation goes here

    stanceXYZ = stanceXYZ - robotCoM;

    numLegs = size(stanceXYZ,2);
    footWrenches = zeros(6,numLegs);
    
    robotWeight = 9.8 * robotMass;

    gravityVecs = repmat(gravityVec,1,numLegs);
    stanceDots = repmat(gravityVec'*stanceXYZ,3,1);
    footDists = sqrt(sum((stanceDots.*gravityVecs - stanceXYZ).^2));

    footCoeffs = sum(footDists) ./ footDists;
    footCoeffs(isFlight) = 0;
    
    footCoeffs = footCoeffs / sum(footCoeffs);
    
    weightVecs = robotWeight * gravityVecs;
    footWrenches(1:3,:) = weightVecs;
    
    gravityPt1 = [0; 0; 0];
    gravityPt2 = gravityVec;
    
    footPt1 = stanceXYZ(:,1);
    footPt2 = stanceXYZ(:,2);
    
    if ~any(isFlight)
        [dist, distVec] = distBetween2Segment( gravityPt1, gravityPt2, ...
                                             footPt1, footPt2 );
        
        springGain = -10;  
        forceVec = springGain * repmat(distVec,1,numLegs);
        forceVec(3) = 0;
        
        footWrenches(1:3,:) = footWrenches(1:3,:) + forceVec;                                  
        torqueVec = cross(distVec,gravityVec);
    else
        torqueVec = zeros(3,1);
    end
    
    footWrenches(4:6,:) = repmat(torqueVec,1,numLegs) * robotWeight; 
    
    footWrenches = repmat(footCoeffs,6,1) .* footWrenches;
end

