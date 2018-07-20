function [ params, trajGen ] = setupOmniBase()
%SETUPOMNIBASE initializes the kinematic and control parameters for a
%3-wheeled omni-drive base.
%
% FRAME CONVENTION:
% ORIGIN = MID-POINT OF THE CHASSIS WHERE ALL 3 WHEEL AXES INTERSECT
% +X-AXIS = FORWARD
% +Y-AXIS = LEFT
% +Z-AXIS = UP

wheelRadius = .150 / 2;  % [m]
wheelBase = .470;   % [m] (diameter of circumscribing circle of the wheels)

params.wheelRadius = wheelRadius;
params.wheelBase = wheelBase;
params.maxLinSpeed = .6; % [m/s]
params.maxRotSpeed = params.maxLinSpeed / (wheelBase/2); % [rad/s]

params.chassisCoM = [0; 0; wheelRadius + 0.005];  % [m]
params.chassisMass = 12;  % kg

% For rotational inertia, assume the robot is a uniform disk
params.chassisInertiaZZ = (1/2) * params.chassisMass * (wheelBase/2)^2;

params.wheelModuleNames = { '_Wheel1', ...
                            '_Wheel2', ...
                            '_Wheel3' };
params.numWheels = 3;

% Load the gains for the wheels
params.wheelGains = HebiUtils.loadGains('gains/omni-drive-wheel-gains');

% Setup the transforms from the origin of the chassis to each wheel,
% starting with the angles that the output axes point out at
a1 = deg2rad(-60);       % Front Right (_Wheel1)
a2 = a1 + deg2rad(120);  % Front Left (_Wheel2)
a3 = a2 + deg2rad(120);  % Rear (_Wheel3)

wheelBaseFrames(:,:,1) = eye(4);
wheelBaseFrames(:,:,2) = eye(4);
wheelBaseFrames(:,:,3) = eye(4);

wheelBaseFrames(1:3,4,1) = [(wheelBase/2)*cos(a1); (wheelBase/2)*sin(a1); wheelRadius];
wheelBaseFrames(1:3,4,2) = [(wheelBase/2)*cos(a2); (wheelBase/2)*sin(a2); wheelRadius];
wheelBaseFrames(1:3,4,3) = [0; -1*(wheelBase/2); wheelRadius];

wheelBaseFrames(1:3,1:3,1) = R_z(a1) * R_y(pi/2);
wheelBaseFrames(1:3,1:3,2) = R_z(a2) * R_y(pi/2);
wheelBaseFrames(1:3,1:3,3) = R_z(a3) * R_y(pi/2);

params.wheelBaseFrames = wheelBaseFrames;

% Maps XYZ chassis velocities to wheel velocities
wheelTransform = [ sin(a1) -cos(a1)  (wheelBase/2);   % Front Right (_Wheel1)
                   sin(a2) -cos(a2)  (wheelBase/2);   % Front Left (_Wheel2)
                   sin(a3) -cos(a3)  (wheelBase/2) ]; % Rear (_Wheel3)

params.wheelVelocityMatrix = wheelTransform ./ wheelRadius;
params.wheelEffortMatrix = wheelTransform * wheelRadius;                          

% Trajectory generator
trajGen = HebiTrajectoryGenerator();
params.rampTime = 0.33;

end