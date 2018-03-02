function [param,kin,trajGen] = setupEdward()
%SETUPEDWARD creates various models for the Edward kit

% Initialize structs
param = struct(); % various constant parameters
kin = struct(); % HebiKinematics objects for all limbs
trajGen = struct(); % HebiTrajectoryGenerator objects for all limbs
speedFactor = 1.0; % [0-1] where 1 is the fastest

%% %%%%%%%%%%%%%%%%%%%%%
% Omni Base Kinematics %
%%%%%%%%%%%%%%%%%%%%%%%%

wheelRadius = .150 / 2;  % m
wheelBase = .235;   % m (center of omni to origin of base)
param.maxLinSpeed = .35; % m/s
param.maxRotSpeed = param.maxLinSpeed / wheelBase; % rad/s

param.chassisCoM = [0; 0; wheelRadius + 0.005];  % m
param.chassisMass = 12;  % kg

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

param.wheelBaseFrames = wheelBaseFrames;

% Maps XYZ chassis velocities to wheel velocities
param.chassisToWheelVelocities = ...
    [0.50 -0.866 wheelBase;
    0.50 0.866 wheelBase;
    -1.00 0.00 wheelBase] ./ wheelRadius;

%% %%%%%%%%%%%%%%
% Chassis Setup %
%%%%%%%%%%%%%%%%%

% Dummy kinematics to initialize trajectory generator with correct 
% velocity limits
chassisKin = HebiKinematics();
chassisKin.addBody('X8-3');
chassisKin.addBody('X8-3');
chassisKin.addBody('X8-3');

% Trajectory generator
trajGen.CHASSIS = HebiTrajectoryGenerator(chassisKin);
trajGen.CHASSIS.setSpeedFactor(speedFactor);

%% %%%%%%%%%%%%
% Spine Setup %
%%%%%%%%%%%%%%%

% Tube Lengths (Spine/Torso)
param.lowerSpineLink = 0.325; % m
param.upperSpineLink = 0.370; % m

% Setup Spine
kin.SPINE = HebiKinematics();
kin.SPINE.addBody('X8-9');
kin.SPINE.addBody('X5-Link','ext',param.lowerSpineLink,'twist',pi);
kin.SPINE.addBody('X8-9');
kin.SPINE.addBody('X5-Link','ext',param.upperSpineLink,'twist',-pi/2);

% Set base frame
spineAnkleFrame(:,:) = eye(4);
spineAnkleFrame(1:3,4) = [.020; 0; .245];
spineAnkleFrame(1:3,1:3) = R_z(pi/2)*R_x(pi/2);
kin.SPINE.setBaseFrame(spineAnkleFrame);

% Trajectory Generator
trajGen.SPINE = HebiTrajectoryGenerator(kin.SPINE);
trajGen.SPINE.setSpeedFactor(speedFactor);
trajGen.SPINE.setAlgorithm('UnconstrainedQp');

%% %%%%%%%%%%
% Arm Setup %
%%%%%%%%%%%%%

param.numArms = 2;
kin.ARMS = cell(1,param.numArms);
trajGen.ARMS = cell(1,param.numArms);

mountingHeavy = {'left-inside','right-inside'};
param.armDir = {-1, 1}; % left then right

for arm = 1:param.numArms
    
    TBodyRot = R_z(param.armDir{arm}*deg2rad(90));
    TBodyFrame = eye(4);
    TBodyFrame(1:3,1:3) = TBodyRot;
    
    % Kinematics
    kin.ARMS{arm} = HebiKinematics();
    kin.ARMS{arm}.addBody('GenericLink','CoM',[0 0 0],'out',TBodyFrame,'mass',.200);
    kin.ARMS{arm}.addBody('X5-Link','ext',.175,'twist',param.armDir{arm}*pi/2);
    kin.ARMS{arm}.addBody('X5-9');
    kin.ARMS{arm}.addBody('X5-HeavyBracket', 'mount', mountingHeavy{arm} );
    kin.ARMS{arm}.addBody('X5-9');
    kin.ARMS{arm}.addBody('X5Link','ext',.325,'twist',0);
    kin.ARMS{arm}.addBody('X5-4');
    kin.ARMS{arm}.addBody('X5Link','ext',.325,'twist',pi);
    kin.ARMS{arm}.addBody('X5-4');
    
    % Trajectory generators
    trajGen.ARMS{arm} = HebiTrajectoryGenerator(kin.ARMS{arm});
    trajGen.ARMS{arm}.setSpeedFactor(speedFactor);
    trajGen.ARMS{arm}.setAlgorithm('UnconstrainedQp');
    
end

end