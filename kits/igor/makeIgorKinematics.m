function [ legKin, armKin, chassisKin ] = makeIgorKinematics(numLegs, numArms)
%MAKEIGORKINEMATICS returns kinematics for the legs, arms, and chassis of
% the X-Series Igor Robot
%
% Andrew Willig
% Nov 2020
%
% Left Arm and Leg = 1
% Right Arm and Leg = 2
% Forward = X
% Left = Y
% Up = Z

% Get relative path
localDir = fileparts(mfilename('fullpath'));

%%%%%%%%%%%%%%%%%%
% Leg Kinematics %
%%%%%%%%%%%%%%%%%%
R_hip = R_x(pi/2);
xyz_hip = [0; .0225; .055];
T_hip = eye(4);
T_hip(1:3,1:3) = R_hip;
T_hip(1:3,4) = xyz_hip;

legBaseFrames(:,:,1) = eye(4);
legBaseFrames(:,:,2) = eye(4);

legBaseFrames(1:3,4,1) = [0; .15; 0];
legBaseFrames(1:3,4,2) = [0; -.15; 0];

legBaseFrames(1:3,1:3,1) = R_x(-pi/2);
legBaseFrames(1:3,1:3,2) = R_x(pi/2);

for leg = 1:numLegs
    legKin{leg} = HebiUtils.loadHRDF([localDir '/hrdf/igorLeg.hrdf']);
    
    legKin{leg}.setBaseFrame(legBaseFrames(:,:,leg));
end


%%%%%%%%%%%%%%%%%%
% Arm Kinematics %
%%%%%%%%%%%%%%%%%%
% 1 = Left, 2 = Right
armBaseXYZ(:,1) = [0; .10; .20];
armBaseXYZ(:,2) = [0; -.10; .20];

armKin{1} = HebiUtils.loadHRDF([localDir '/hrdf/igorArm-Left.hrdf']);
armKin{2} = HebiUtils.loadHRDF([localDir '/hrdf/igorArm-Right.hrdf']);

for arm = 1:numArms
    armTransform = eye(4);
    armTransform(1:3,4) = armBaseXYZ(:,arm);
    armKin{arm}.setBaseFrame(armTransform);
end

%%%%%%%%%%%%%%%%%%%%%%
% Chassis Kinematics %
%%%%%%%%%%%%%%%%%%%%%%
chassiskin = HebiUtils.loadHRDF([localDir '/hrdf/igorChassis.hrdf']);