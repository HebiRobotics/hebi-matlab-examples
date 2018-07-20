function [ params, trajGen ] = setupDiffDriveBase()
%SETUPDIFFDRIVEBASE initializes the kinematic and control parameters for a
%2-wheeled diff-drive base.
%
% FRAME CONVENTION:
% ORIGIN = MID-POINT BETWEEN THE WHEELS
% +X-AXIS = FORWARD
% +Y-AXIS = LEFT
% +Z-AXIS = UP

wheelDiameter = .200; % [m]
wheelBase = .430; % [m]

wheelRadius = wheelDiameter / 2;  % [m]

% Fill in a struct with all this information to return out into the main
% demo script.
params.wheelRadius = wheelRadius;
params.wheelBase = wheelBase;
params.maxLinSpeed = 0.8; % m/s
params.maxRotSpeed = params.maxLinSpeed / wheelBase; % rad/s

params.chassisCoM = [0; 0; wheelRadius + 0.005];  % m
params.chassisMass = 10;  % kg

% For rotational inertia, assume the robot is a uniform disk
params.chassisInertiaZZ = (1/2) * params.chassisMass * (wheelBase/2)^2;

params.wheelNames = { '_LeftWheel', ...
                      '_RightWheel' };
params.numWheels = 2;

% Load the gains for the wheels
params.wheelGains = HebiUtils.loadGains('gains/diff-drive-wheel-gains');

% Transform from the origin of the chassis to the Left Wheel
wheelBaseFrames(:,:,1) = eye(4); 
wheelBaseFrames(1:3,4,1) = [ 0; 
                             wheelBase/2; 
                             0 ];
wheelBaseFrames(1:3,1:3,1) = R_z(pi/2) * R_y(pi/2);

% Transform from the origin of the chassis to the Right Wheel
wheelBaseFrames(:,:,2) = eye(4); % right
wheelBaseFrames(1:3,4,2) = [ 0; 
                            -wheelBase/2; 
                             0 ];             
wheelBaseFrames(1:3,1:3,2) = R_z(-pi/2) * R_y(pi/2);

params.wheelBaseFrames = wheelBaseFrames;

% Setup a matrix to map XYZ chassis velocities to wheel velocities
wheelMatrix = [ 1 0 wheelBase/2;    % Left Wheel (_LeftWheel)
               -1 0 wheelBase/2 ];  % Right Wheel (_RightWheel)

params.wheelVelocityMatrix = wheelMatrix ./ wheelRadius;
params.wheelEffortMatrix = wheelMatrix * wheelRadius;      

% Trajectory generator
trajGen = HebiTrajectoryGenerator();

end