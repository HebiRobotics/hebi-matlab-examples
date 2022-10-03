function [ params, trajGen ] = setupPneumDriveBase()
%SETUPPNEUMDRIVEBASE initializes the kinematic and control parameters for a
%4-wheeled diff-drive base.
%
% FRAME CONVENTION:
% ORIGIN = MID-POINT BETWEEN THE WHEELS
% +X-AXIS = FORWARD
% +Y-AXIS = LEFT
% +Z-AXIS = UP

wheelDiameter = .185; % [m]
wheelBase = .445; % [m]
wheelBaseLength = 0.415;

wheelRadius = wheelDiameter / 2;  % [m]

% Fill in a struct with all this information to return out into the main
% demo script.
params.wheelRadius = wheelRadius;
params.wheelBase = wheelBase;
params.maxLinSpeed = 0.4; %0.8; % m/s
params.maxRotSpeed = params.maxLinSpeed / wheelBase; % rad/s

params.chassisCoM = [0; 0; wheelRadius];  % m
params.chassisMass = 10;  % kg

% For rotational inertia, assume the robot is a uniform disk
params.chassisInertiaZZ = (1/2) * params.chassisMass * (wheelBase/2)^2;

params.wheelModuleNames = { 'W1_front-left', ...
                            'W2_front-right' ...
                            'W3_rear-left' ...
                            'W4_rear-right' };
params.numWheels = 4;

% Load the gains for the wheels
params.wheelGains = HebiUtils.loadGains('gains/pneum-drive-wheel-gains');

% Transform from the origin of the chassis to the Front Left Wheel
wheelBaseFrames(:,:,1) = eye(4); 
wheelBaseFrames(1:3,4,1) = [ wheelBaseLength/2; 
                             wheelBase/2; 
                             0 ];
wheelBaseFrames(1:3,1:3,1) = R_z(pi/2) * R_y(pi/2);

% Transform from the origin of the chassis to the Rear Left Wheel
wheelBaseFrames(:,:,2) = eye(4); 
wheelBaseFrames(1:3,4,2) = [ -wheelBaseLength/2; 
                             wheelBase/2; 
                             0 ];
wheelBaseFrames(1:3,1:3,2) = R_z(pi/2) * R_y(pi/2);

% Transform from the origin of the chassis to the Front Right Wheel
wheelBaseFrames(:,:,3) = eye(4); % right
wheelBaseFrames(1:3,4,3) = [ wheelBaseLength/2; 
                            -wheelBase/2; 
                             0 ];             
wheelBaseFrames(1:3,1:3,3) = R_z(-pi/2) * R_y(pi/2);

params.wheelBaseFrames = wheelBaseFrames;

% Transform from the origin of the chassis to the Rear Right Wheel
wheelBaseFrames(:,:,4) = eye(4); % right
wheelBaseFrames(1:3,4,4) = [ -wheelBaseLength/2; 
                            -wheelBase/2; 
                             0 ];             
wheelBaseFrames(1:3,1:3,4) = R_z(-pi/2) * R_y(pi/2);


params.wheelBaseFrames = wheelBaseFrames;

% Setup a matrix to map XYZ chassis velocities to wheel velocities
% Maps XYZ chassis velocities to wheel velocities
wheelMatrix = [ 1.0, 0.5; % Front Left
                    -1.0, 0.5; % Rear Left
                    1.0, 0.5; % Front Right
                    -1.0, 0.5 ]; % Rear Right

params.wheelVelocityMatrix = wheelMatrix ./ wheelRadius;
params.wheelEffortMatrix = wheelMatrix * wheelRadius;      

% Trajectory generator
trajGen = HebiTrajectoryGenerator();
params.rampTime = 0.33;

end
