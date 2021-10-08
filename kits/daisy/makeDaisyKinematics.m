function [ legKin, chassisKin ] = makeDaisyKinematics( )
%MAKEHEXAPOD returns kinematics for each leg of the X-Series Hexapod
%
% Dave Rollinson
% Feb 2017
%
%  Leg Numbering / Chassis Coordinate convention:
%  This should match ROS wheeled vehicle convention
%  
%    1 --- 2         +x
%       |             ^
%   3 ----- 4         |
%       |       +y <--o
%    5 --- 6          +z


% Get relative path
localDir = fileparts(mfilename('fullpath'));
    
% Distances updated on 1 Apr 2019
chassisRadii = [0.2375 0.2375 0.1875 0.1875 0.2375 0.2375]; % m
chassisAngles = deg2rad( [30 -30 90 -90 150 -150] );

chassiskin = HebiUtils.loadHrdf([localDir '/hrdf/daisyChassis.hrdf']);

baseLims = [-pi/3 pi/3];
shoulderLims = [-pi/2 pi/3];
elbowLims = [-pi pi/4];

for leg=1:6
    if rem(leg,2)==1
        legKin{leg} = HebiUtils.loadHrdf([localDir '/hrdf/daisyLeg-Left.hrdf']);
    else    
        legKin{leg} = HebiUtils.loadHrdf([localDir '/hrdf/daisyLeg-Right.hrdf']);
    end

    legTransform = eye(4);
    legTransform(1:3,1:3) = R_z(chassisAngles(leg));
    legTransform(1:3,4) = ...
                R_z(chassisAngles(leg)) * [chassisRadii(leg); 0; 0];
    legKin{leg}.setBaseFrame(legTransform);
end

