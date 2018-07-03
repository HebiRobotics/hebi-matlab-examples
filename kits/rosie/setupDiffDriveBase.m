function [ params, trajGen ] = setupDiffDriveBase()
%SETUPEDWARD creates various models for the Edward kit

% Initialize structs
params = struct(); % various constant parameters

%% %%%%%%%%%%%%%%%%%%%%%
% Omni Base Kinematics %
%%%%%%%%%%%%%%%%%%%%%%%%

wheelRadius = .200 / 2;  % m
wheelBase = .430;   % m (center chassis to each wheel)

params.wheelRadius = wheelRadius;
params.wheelBase = wheelBase;
params.maxLinSpeed = 0.8; % m/s
params.maxRotSpeed = params.maxLinSpeed / wheelBase; % rad/s

params.chassisCoM = [0; 0; wheelRadius + 0.005];  % m
params.chassisMass = 10;  % kg

params.numWheels = 3;

wheelBaseFrames(:,:,1) = eye(4); % left
wheelBaseFrames(:,:,2) = eye(4); % right
wheelBaseFrames(1:3,4,1) = [0; wheelBase/2; 0];
wheelBaseFrames(1:3,4,2) = [0; -wheelBase/2; 0];
wheelBaseFrames(1:3,1:3,1) = R_z(pi/2) * R_y(pi/2);
wheelBaseFrames(1:3,1:3,2) = R_z(-pi/2) * R_y(pi/2);

params.wheelBaseFrames = wheelBaseFrames;

% Maps XYZ chassis velocities to wheel velocities
%
% FRAME CONVENTION (FROM THE "DRIVER'S SEAT"):
% +X-AXIS = FORWARD
% +Y-AXIS = LEFT
% +Z-AXIS = UP
wheelTransform = [ 1 0 wheelBase;    % Left Wheel (_LeftWheel)
                  -1 0 wheelBase ];  % Right Wheel (_RightWheel)

params.wheelVelocityMatrix = wheelTransform ./ wheelRadius;
params.wheelEffortMatrix = wheelTransform * wheelRadius;      


% Trajectory generator
trajGen = HebiTrajectoryGenerator();

end