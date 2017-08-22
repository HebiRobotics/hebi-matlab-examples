%% HebiKinematics
% This example shows howto use the HebiKinematics API to calculate forward
% kinematics, Jacobians, as well as forces and torques to compensate
% for accelerations due to gravity and the dynamics.
%
%%
%
% <html>
% <table border=0>
%   <tr><td>Created</td><td>July 13, 2017</td></tr>
%   <tr><td>Last Update</td><td>Aug 22, 2017</td></tr>
%   <tr><td>API Version</td><td>hebi-matlab-1.0-rc2</td></tr>
%   <tr><td>Requirements</td><td>MATLAB 2013b or higher</td></tr>
% </table>
% </html>
%
% Copyright 2017 HEBI Robotics

%% Define kinematic structure using HEBI components
% The addBody method creates a serial chain of bodies that describe the 
% kinematic relation of a robot. A body can be a rigid link or a 
% dynamic element. The first body represents the base and the last body 
% represents the end-effector.
%
% The following code creates a representation of the rendered 5-DoF arm  
% shown below. For more information on available bodies, please consult
% the help files and/or the online documentation at <http://docs.hebi.us>.
%
% <<resources/5dof_fk.png>>
%

% Setup 5dof arm
kin = HebiKinematics();
kin.addBody('X5-4'); % base joint
kin.addBody('X5-HeavyBracket', 'mount', 'right-inside');
kin.addBody('X5-9');
kin.addBody('X5-Link', 'ext', 0.350, 'twist', pi);
kin.addBody('X5-9');
kin.addBody('X5-Link', 'ext', 0.250, 'twist', pi);
kin.addBody('X5-1');
kin.addBody('X5-LightBracket', 'mount', 'left');
kin.addBody('X5-1');

% Display
display(kin);

%% Calculate Forward Kinematics
% While the HebiKinematics and the HebiGroup APIs were desigend to work
% well together, the HebiKinematics API is independent and can be used
% by itself. To keep these examples simple, we use pre-defined position
% vectors. In a real application, positions can be set to the group
% feedback.

% Input position vector (could be replaced with "position = fbk.position")
position = rand(1, kin.getNumDoF);
display(position);

% 4x4xN transforms from the base frame to each output
frames = kin.getForwardKinematics('output', position);

% 4x4 transform from the base frame to the end-effector
endEffector = kin.getForwardKinematicsEndEffector(position);
display(endEffector);


%% Calculate Jacobian
% The calls to get the Jacobian work the same way as the forward
% kinematics.

% 6x numDoF x numBodies matrix
J = kin.getJacobian('output', position);

% 6 x numDoF matrix
J_endEffector = kin.getJacobianEndEffector(position);
display(J_endEffector);

%% Gravity Compensation
% There are many use cases, such as teach-repeat, that benefit from an arm
% being in a compliant 'zero-gravity' mode. The Kinematics API provides
% convenience methods that help with calculating the required forces and 
% torques to counter gravity.

% Calculate compensatory torques/forces at position zero w/ gravity 
% pointing in the negative z direction
gravityVec = [0 0 -1];
position = zeros(1, kin.getNumDoF);
gravCompEfforts = kin.getGravCompEfforts(position, gravityVec);

%%%
% The following example continuously compensates for gravity on a 2 dof
% arm. It works with any serial chain robot configuration provided
% that the actuators in the group match the kinematic configuration.
%
% <<resources/2dof_RR_ik_xy.png>>
%

% Setup 2 dof planar RR arm (could be any number of DoF)
kin = HebiKinematics();
kin.addBody('X5-4'); % base joint
kin.addBody('X5-Link', 'ext', 0.35, 'twist', pi);
kin.addBody('X5-1');
kin.addBody('X5-Link', 'ext', 0.25, 'twist', pi);

% Setup the group that corresponds to the cnfiguration
group = HebiLookup.newGroupFromNames('arm', {'base', 'shoulder'});

% Determine the direction of gravity based on the built-in IMU 
% (assumes fixed base)
fbk = group.getNextFeedback();
gravityVec = -[fbk.accelX(1) fbk.accelY(1) fbk.accelZ(1)];

% Continuously command a 'weight-less' mode
t0 = tic();
cmd = CommandStruct();
while toc(t0) < 5
    fbk = group.getNextFeedback();
    cmd.effort = kin.getGravCompEfforts(fbk.position, gravityVec);
    group.send(cmd);
end

%% Dynamics Compensation
% Similarly to gravity compensation, HebiKinematics also provides a way to
% calculate forces and torques to compensate for the joint accelerations 
% due to dynamic motions.
%
% This does require knowledge of positions, velocities, and accelerations
% at a given point, so it is primarily useful when being combined with a
% trajectory, such as provided by the HebiTrajectory API (later example).
%
% The dynamics compensation does not include the torques/forces required
% to compensate for gravity. Thus, getDynamicCompEfforts() is typically
% used in combination with getGravCompEfforts().

% Create sample sinusoidal motion
time = 0.2; 
freq = 1 * (2*pi);  % 1 Hz 
amp = 1; 
allDoF = ones(1,group.getNumModules);

% Sample point in trajectory
cmdPosition = amp * sin( freq * time ) * allDoF;
cmdVelocity = freq * amp * cos( freq * time ) * allDoF;
cmdAcceleration = -freq^2 * amp * sin( freq * time ) * allDoF;

% Calculate torques at sampled point within trajectory
efforts = kin.getDynamicCompEfforts(...
    fbk.position, ...
    cmdPosition, ...
    cmdVelocity, ...
    cmdAcceleration);

% Display
display(cmdPosition);
display(cmdVelocity);
display(cmdAcceleration);
display(efforts);

