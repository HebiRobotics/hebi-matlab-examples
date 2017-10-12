function [ totalError ] = DCM_Error( ...
                    DCM_Target, jointAngs, fwdKinFunc, armBaseFrame )
% Returns the error of the XYZ position with respect a target XYZ point 
% that gets passed in.
%
% This function is meant to be called by LSQNONLIN.
%
% Includes a tweak to keep the joints away from joint limits.  Basically it
% adds an error that penalizes being close to the limits.  By using a
% really high (even-numbered!) exponent we make an error funciton that only
% ramps up severely at the joints.
%
% Dave Rollinson
% Sep 2012

    %%%%%%%%%%%%%
    % XYZ Error %
    %%%%%%%%%%%%%
    
    % Forward Kinematics of the youBot arm
    robotFrames = fwdKinFunc( jointAngs, armBaseFrame );
    
    % Pull the XYZ coordinates from robotFrames
    % The last 4x4 transform is the pose of the end effector
    DCM_EndEffector = robotFrames(1:3,1:3,end);
    
    % Subtract to get the error
    SO3_Error = DCM_Target' * DCM_EndEffector;
    
    % Off-Diagonals of the matrix is the error to minimize
    DCM_Error = [ SO3_Error(1,2) SO3_Error(1,3) SO3_Error(2,3) ]'; 
    
    
    %%%%%%%%%%%%%%%%%%%%%
    % Joint Limit Error %
    %%%%%%%%%%%%%%%%%%%%%

    % Coefficients on penalizing being close to joint limits
    limScale = 5;
    limPower = 50;
    
    limError = limScale * (jointAngs(2:end) / (pi/2)).^limPower;
                 
    
    %%%%%%%%%%%%%%%
    % Total Error %
    %%%%%%%%%%%%%%%
    
    % XYZ and Joint Limit Errors
    totalError = [ DCM_Error; limError' ];

end

