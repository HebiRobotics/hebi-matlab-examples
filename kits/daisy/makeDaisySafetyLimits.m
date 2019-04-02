function [ safetyParams ] = makeDaisySafetyLimits( )
% This function makes a SafetyParamsStruct to set the joing position limits
% on the robot.  This will eventually be replaced by loading from an XML
% file.
%
% Dave Rollinson
% Apr 2019

    safetyParams = SafetyParamsStruct();
    
    safetyParams.mStopStrategy = 2 * ones(1,18); % 0 = DISABLED
                                                 % 1 = MOTOR OFF
                                                 % 2 = MOTOR HOLD
                                             
    safetyParams.positionLimitStrategy = 3 * ones(1,18);  % 0 = DISABLED
                                                          % 1 = MOTOR OFF
                                                          % 2 = HOLD POSITION
                                                          % 3 = DAMPED SPRING
    
    % Base position limits same for all legs
    baseMinPosLimits = -pi/2 * ones(1,6);
    baseMaxPosLimits = pi/2 * ones(1,6);
    
    % Shoulder position limits switch sign depending left or right leg
    shoulderMinPosLimits = [ -pi/2 -pi/4  pi/2 -pi/4 -pi/2 -pi/4 ];
    shoulderMaxPosLimits = [  pi/4  pi/2  pi/4  pi/2  pi/4  pi/2 ];
    
    % Elbow position limits switch sign depending left or right leg
    elbowMinPosLimits = [ -pi  0  -pi  0  -pi  0  ];
    elbowMaxPosLimits = [  0   pi  0   pi  0   pi ];
    
    % Stack the limits for each joint and then vectorize them to put them
    % in the correct order for the entire robot.
    positionMinLimits = [ baseMinPosLimits;
                          shoulderMinPosLimits;
                          elbowMinPosLimits ];
    positionMaxLimits = [ baseMaxPosLimits;
                          shoulderMaxPosLimits;
                          elbowMaxPosLimits ]; 
                      
    safetyParams.positionMinLimit = positionMinLimits(:)';    
    safetyParams.positionMaxLimit = positionMaxLimits(:)';    
end