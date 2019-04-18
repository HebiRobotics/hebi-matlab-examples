function [ springEfforts ] = getGasSpringAssist( legFrames, J_leg, springParams )
%GETGASSPRINGASSIST takes in frames that describe the forward kinematics of
%a leg for Florence and calculates the torques that a gas spring provides
%to the hips and knee.

    % Actuator frames from FK
    hipFrameNum = 8;
    kneeFrameNum = 13;
    hipInput = legFrames(:,:,hipFrameNum);
    kneeOutput = legFrames(:,:,kneeFrameNum);
    
    springVector = (kneeOutput(1:3,4) - hipInput(1:3,4));
    springLength = norm(springVector);
    springVector = springVector / springLength;
    
    % Adjust spring force with the spring length
    compressRatio = 0.5 * (springParams.freeLength - springLength) / ...
                                        springParams.freeLength;
    
    adjustedForce = springParams.baseForce + ...
                        compressRatio * springParams.baseForce;
    
    springWrench = zeros(6,1);
    springWrench(1:3) = -adjustedForce * springVector;
    

    % MAYBE DO SOMETHING WITH THE SPRING VELOCITY
    
    springEfforts = J_leg(:,:,kneeFrameNum)' * springWrench;
end

