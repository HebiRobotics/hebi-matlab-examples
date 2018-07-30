function [ kin ] = makeHexapodKinematics( )
%MAKEHEXAPOD returns kinematics for each leg of the X-Series Hexapod
%
% Dave Rollinson
% Feb 2017
%
%  Leg Numbering / Chassis Coordinate convention:
%  This should match ROS wheeled vehicle convention
%  
%   1 ----- 2         +x
%       |             ^
%   3 ----- 4         |
%       |       +y <--o
%   5 ----- 6          +z

% Distances based on CAD on 22 Feb 2017
chassisRadii = [.187 .187 .136 .136 .187 .187]; % m
chassisAngles = deg2rad( [30 -30 90 -90 150 -150] );

R_hip = R_x(-pi/2);
xyz_hip = [0; -.0225; .055]; 
T_hip = eye(4);
T_hip(1:3,1:3) = R_hip;
T_hip(1:3,4) = xyz_hip;

baseLims = [-pi/3 pi/3];
shoulderLims = [-pi/2 pi/3];
elbowLims = [-pi pi/4];

for leg=1:6
    kin{leg} = HebiKinematics();
    kin{leg}.addBody('X5-4','posLim',baseLims);
    %kin{leg}.addBody('X5Bracket');
    kin{leg}.addBody( 'GenericLink', 'com', xyz_hip/2, 'out', T_hip, 'mass', .150 );
    kin{leg}.addBody('X5-4','posLim',shoulderLims);
    kin{leg}.addBody('X5Link','ext',.279,'twist',pi);
    kin{leg}.addBody('X5-4','posLim',elbowLims);
    kin{leg}.addBody('X5Link','ext',.276,'twist',0);

    legTransform = eye(4);
    legTransform(1:3,1:3) = R_z(chassisAngles(leg));
    legTransform(1:3,4) = ...
                R_z(chassisAngles(leg)) * [chassisRadii(leg); 0; 0];
    kin{leg}.setBaseFrame(legTransform);
end

