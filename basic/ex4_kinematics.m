% This example shows howto use the HebiKinematics API to calculate forward
% kinematics and Jacobians.
% 
% Requirements:  MATLAB 2013b or higher
%
% Author:        Florian Enner
% Created:       13 July, 2017
% API:           hebi-matlab-1.0
%
% Copyright 2017 HEBI Robotics

%% Define kinematic structure using HEBI components
% The addBody method creates a serial chain of bodies that describe the 
% kinematic relation of a robot. A 'body' can be a rigid link as well as a 
% dynamic element. The first body represents the base and the last body 
% represents the end-effector.
%
% The following code creates a representation of the rendered 5-DoF arm  
% shown below. For more information on the available parts, please consult
% the help files and/or the online documentation.
%
% <<resources/5dof_fk.png>>
%

% Setup
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

%% Programmatically access body info
bodyInfo = kin.getBodyInfo();
display(bodyInfo);

%% Programmatically access joint info
jointInfo = kin.getJointInfo();
display(jointInfo);

%% Get Forward kinematics
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


%% Get Jacobian
% The calls to get the Jacobian work the same way as the forward
% kinematics.

% 6x numDoF x numBodies matrix
J = kin.getJacobian('output', position);

% 6 x numDoF matrix
J_endEffector = kin.getJacobianEndEffector(position);
display(J_endEffector);
