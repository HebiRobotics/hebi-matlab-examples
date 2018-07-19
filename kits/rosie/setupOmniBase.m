function [ params, trajGen ] = setupOmniBase()
%SETUPEDWARD creates various models for the Edward kit

% Initialize structs
params = struct(); % various constant parameters
%% %%%%%%%%%%%%%%%%%%%%%
% Omni Base Kinematics %
%%%%%%%%%%%%%%%%%%%%%%%%

wheelRadius = .150 / 2;  % m
wheelBase = .235;   % m (center of omni to origin of base)

params.wheelRadius = wheelRadius;
params.wheelBase = wheelBase;
params.maxLinSpeed = .75; % m/s
params.maxRotSpeed = params.maxLinSpeed / wheelBase; % rad/s

params.chassisCoM = [0; 0; wheelRadius + 0.005];  % m
params.chassisMass = 12;  % kg

params.numWheels = 3;

% Setup angles with each wheel
a1 = pi/6;          % rad (30 deg)
a2 = a1 + 2*pi/3;   % rad (150 deg)
a3 = a2 + 2*pi/3;   % rad (270 deg)

wheelBaseFrames(:,:,1) = eye(4);
wheelBaseFrames(:,:,2) = eye(4);
wheelBaseFrames(:,:,3) = eye(4);

wheelBaseFrames(1:3,4,1) = [wheelBase*cos(a1); wheelBase*sin(a1); wheelRadius];
wheelBaseFrames(1:3,4,2) = [wheelBase*cos(a2); wheelBase*sin(a2); wheelRadius];
wheelBaseFrames(1:3,4,3) = [0; -1*wheelBase; wheelRadius];

wheelBaseFrames(1:3,1:3,1) = R_z(a1)*R_y(pi/2);
wheelBaseFrames(1:3,1:3,2) = R_z(a2)*R_y(pi/2);
wheelBaseFrames(1:3,1:3,3) = R_z(a3)*R_y(pi/2);

params.wheelBaseFrames = wheelBaseFrames;

% Maps XYZ chassis velocities to wheel velocities
<<<<<<< HEAD
%
% FRAME CONVENTION (FROM THE "DRIVER'S SEAT"):
% +X-AXIS = FORWARD
% +Y-AXIS = LEFT
% +Z-AXIS = UP
wheelTransform = [ -0.866 -0.500  wheelBase;   % Front Right (_Wheel1)
                    0.866 -0.500  wheelBase;   % Front Left (_Wheel2)
                    0.000  1.000  wheelBase ]; % Rear Wheel (_Wheel3)
=======
wheelTransform = [ 0.50 -0.866 wheelBase;
                   0.50  0.866 wheelBase;
                  -1.00  0.00  wheelBase ];
>>>>>>> master

params.wheelVelocityMatrix = wheelTransform ./ wheelRadius;
params.wheelEffortMatrix = wheelTransform * wheelRadius;                          

% Trajectory generator
trajGen = HebiTrajectoryGenerator();

end