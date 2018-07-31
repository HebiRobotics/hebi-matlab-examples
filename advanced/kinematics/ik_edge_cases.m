% Setup robot kinematics, calculate inverse kinematics (IK) to several points in
% and outside workspace, and check error to see if solution was valid
%
% For more information type:
%    help HebiKinematics
%
% HEBI Robotics
% July 2018

%%
clear *;
close all;

% Load the kinematics from HRDF file
kin = HebiKinematics('./hrdf/3-DoF_arm_example.hrdf');

% Initialize a helper utility to view the various coordinate frames
frameDisp = FrameDisplay();

%%%%%%%%%%%%%%%%%
%% Test case 1:

% Note that this point is inside of the workspace, and the IK succeeds.

% The desired end effector position
targetXYZ = [ 0.5; -0.2; 0.1 ];  % [m]

% initial position to start the local IK search at
initPositions = [-pi/4 pi/4 pi/2];  % "elbow up" solutions

% Solve IK
positions = kin.getInverseKinematics( 'xyz', targetXYZ, ...
                                      'initial', initPositions );

% Check cartesian error by calculating the FK of the IK solution. Here we use
% the distance between our target xyz location and the FK of the IK solution.
eeLocation = kin.getFK('endEffector', positions);
xyzError = sqrt(sum((eeLocation(1:3,4) - targetXYZ).^2));
disp('== Test case 1 (inside workspace, good initial conditions) ==');
disp('Cartesian error (m):');
disp(xyzError);

%%%%%%%%%%%%%%%%%
%% Test case 2:

% Note that this point is outside of the workspace, and so the IK converges to
% a solution that is "as close as possible"

targetXYZ = [ 1.0; -0.2; 0.1 ];  % [m]
initPositions = [-pi/4 pi/4 pi/2];  % "elbow up" solutions
positions = kin.getInverseKinematics( 'xyz', targetXYZ, ...
                                      'initial', initPositions );
eeLocation = kin.getFK('endEffector', positions);
xyzError = sqrt(sum((eeLocation(1:3,4) - targetXYZ).^2));
disp('== Test case 2 (outside workspace) ==');
disp('Cartesian error (m):');
disp(xyzError);

%%%%%%%%%%%%%%%%%
%% Test case 3:

% Note that this initial condition (arm straight up) was chosen as a singularity
% with no differential motion that moves the end effector in the z direction.
% a solution that is "as close as possible"
initPositions = [ 0 -pi/2 0 ];

% We get a target position that is straight down (in Z) from the initial
% cartesian position
eeLocation = kin.getFK('endEffector', initPositions);
targetXYZ = eeLocation(1:3, 4) + [ 0; 0; -0.2 ]; % [m]

% When we compute IK to go to this target location, the local optimization
% cannot find any differential motion that moves closer, and so returns
positions = kin.getInverseKinematics( 'xyz', targetXYZ, ...
                                      'initial', initPositions );
eeLocation = kin.getFK('endEffector', positions);
xyzError = sqrt(sum((eeLocation(1:3,4) - targetXYZ).^2));
disp('== Test case 3 (singularity) ==');
disp('Cartesian error (m):');
disp(xyzError);

%%%%%%%%%%%%%%%%%
%% Test case 4:

% By moving very slightly from our singularity (.001 radians), the solution
% converges immediately

initPositions = [ 0 -pi/2 + .001 0 ];
positions = kin.getInverseKinematics( 'xyz', targetXYZ, ...
                                      'initial', initPositions );
eeLocation = kin.getFK('endEffector', positions);
xyzError = sqrt(sum((eeLocation(1:3,4) - targetXYZ).^2));
disp('== Test case 4 (near singularity) ==');
disp('Cartesian error (m):');
disp(xyzError);
