classdef HebiEnum
    %HEBIENUM maps strategies and states to human readable values
    %
    %  Maps strategy settings and state feedback to human readable
    %  enum values.
    %
    %  Example
    %      fbk = group.getNextFeedbackFull();
    %      if(fbk.temperatureState ~= HebiEnum.TemperatureStateNormal)
    %          disp('reducing performance due to temperature limits!');
    %      end
    %
    %  Example
    %      safetyParams = SafetyParamsStruct();
    %      safetyParams.mStopStrategy = HebiEnum.MStopStrategyMotorHold;
    %      safetyParams.positionLimitStrategy = HebiEnum.PositionLimitStrategyHoldPosition;
    % 
    %  For detailed information about each state, please run 'doc HebiEnum'
    %  or check the online API documentation.
    %
    %  See also FeedbackStruct, SafetyParamsStruct, GainStruct
    
    % Copyright 2019-2019 HEBI Robotics, Inc
    
    properties(Constant)
        
        %% 'controlStrategy'
        
        ControlStrategyNotAvailable = nan; %The strategy is not available or the strategy will not be changed.
        ControlStrategyOff = 0; %The motor is not given power (equivalent to a 0 PWM value)
        ControlStrategyDirectPWM = 1; %A direct PWM value (-1 to 1) can be sent to the motor (subject to onboard safety limiting).
        ControlStrategy2 = 2; %A combination of the position, velocity, and effort loops with P and V feeding to T; documented on docs.hebi.us under "Control Modes"
        ControlStrategy3  = 3; %A combination of the position, velocity, and effort loops with P, V, and T feeding to PWM; documented on docs.hebi.us under "Control Modes"
        ControlStrategy4  = 4; %A combination of the position, velocity, and effort loops with P feeding to T and V feeding to PWM; documented on docs.hebi.us under "Control Modes"
        
        %% 'mStopStrategy'
        
        MStopStrategyNotAvailable = nan; %The strategy is not available or the strategy will not be changed.
        MStopStrategyDisabled = 0; %Triggering the M-Stop has no effect.
        MStopStrategyMotorOff = 1; %Triggering the M-Stop results in the control strategy being set to 'off'. Remains 'off' until changed by user.
        MStopStrategyHoldPosition = 2; %Triggering the M-Stop results in the actuator holding the position. Operations resume to normal once trigger is released.
        
        %% 'positionLimitStrategy'
        
        PositionLimitStrategyNotAvailable = nan; %The strategy is not available or the strategy will not be changed.
        PositionLimitStrategyDisabled = 0; %Triggering the position limit has no effect.
        PositionLimitStrategyMotorOff = 1; %Triggering the position limit results in the control strategy being set to 'off'. Remains 'off' until changed by user.
        PositionLimitStrategyHoldPosition = 2; %Triggering the position limit results in the actuator holding the position. Needs to be manually set to 'disabled' to recover.
        PositionLimitStrategyDampedSpring = 3; %Triggering the position limit results in a virtual spring that pushes the actuator back to within the limits. 
        
        %% 'resetButtonState'
        
        ResetButtonStateNotAvailable = nan; %The state is not available.
        ResetButtonStateNotTriggered  = 0; %The reset button is not pressed.
        ResetButtonStateTriggered  = 1; %The reset button is pressed.
        
        %% 'mStopState'
        
        MStopStateNotAvailable = nan; %The state is not available.
        MStopStateNotTriggered  = 0; %The MStop is not pressed.
        MStopStateTriggered  = 1; %The MStop is pressed.
        
        %% 'temperatureState'
        
        TemperatureStateNotAvailable = nan; %The state is not available.
        TemperatureStateNormal = 0; %Temperature within normal range.
        TemperatureStateCritical = 1; %Motor output beginning to be limited due to high temperature.
        TemperatureStateExceedMaxMotor = 2; %Temperature exceeds max allowable for motor; motor output disabled.
        TemperatureStateExceedMaxBoard = 3; %Temperature exceeds max allowable for electronics; motor output disabled.
        
        %% 'positionLimitState'
        
        PositionLimitStateNotAvailable = nan; %The state is not available.
        PositionLimitStateBelow = -2; %The position of the module was below the lower safety limit; the motor output is set to return the module to within the limits
        PositionLimitStateAtLower = -1; %The position of the module was near the lower safety limit, and the motor output is being limited or reversed.
        PositionLimitStateInside = 0; %The position of the module was within the safety limits.
        PositionLimitStateAtUpper = +1; %The position of the module was near the upper safety limit, and the motor output is being limited or reversed.
        PositionLimitStateAbove = +2; %The position of the module was above the upper safety limit; the motor output is set to return the module to within the limits
        PositionLimitStateUninitialized = 10; %The module has not been inside the safety limits since it was booted or the safety limits were set.
        
        %% 'velocityLimitState'
        
        VelocityLimitStateNotAvailable = nan; %The state is not available.
        VelocityLimitStateBelow = -2; %The velocity of the module was below the lower safety limit; the motor output is set to return the module to within the limits
        VelocityLimitStateAtLower = -1; %The velocity of the module was near the lower safety limit, and the motor output is being limited or reversed.
        VelocityLimitStateInside = 0; %The velocity of the module was within the safety limits.
        VelocityLimitStateAtUpper = +1; %The velocity of the module was near the upper safety limit, and the motor output is being limited or reversed.
        VelocityLimitStateAbove = +2; %The velocity of the module was above the upper safety limit; the motor output is set to return the module to within the limits
        VelocityLimitStateUninitialized = 10; %The module has not been inside the safety limits since it was booted or the safety limits were set.
        
        %% 'effortLimitState'
        
        EffortLimitStateNotAvailable = nan; %The state is not available.
        EffortLimitStateBelow = -2; %The effort of the module was below the lower safety limit; the motor output is set to return the module to within the limits
        EffortLimitStateAtLower = -1; %The effort of the module was near the lower safety limit, and the motor output is being limited or reversed.
        EffortLimitStateInside = 0; %The effort of the module was within the safety limits.
        EffortLimitStateAtUpper = +1; %The effort of the module was near the upper safety limit, and the motor output is being limited or reversed.
        EffortLimitStateAbove = +2; %The effort of the module was above the upper safety limit; the motor output is set to return the module to within the limits
        EffortLimitStateUninitialized = 10; %The module has not been inside the safety limits since it was booted or the safety limits were set.
        
        %% 'commandLockState'
        
        CommandLockStateNotAvailable = nan; %The state is not available.
        CommandLockStateUnlocked = -1; %There is no command lifetime active on this module.
        CommandLockStateLockedBySender = 0; %Commands from others are locked out due to control from this group.
        CommandLockStateLockedByOther = 1; %Commands are locked out due to control from other users.
        
        %% 'arQuality'
        
        ArQualityNotAvailable = nan; %Camera position tracking is not available.
        ArQualityNormal = 0; %Camera position tracking is providing optimal results.
        ArQualityLimitedUnknown = 1; %Tracking is available albeit suboptimal for an unknown reason.
        ArQualityLimitedInitializing = 2; %The AR session has not yet gathered enough camera or motion data to provide tracking information.
        ArQualityLimitedRelocalizing = 3; %The AR session is attempting to resume after an interruption.
        ArQualityLimitedExcessiveMotion = 3; %The device is moving too fast for accurate image-based position tracking.
        ArQualityLimitedInsufficientFeatures = 5; %The scene visible to the camera does not contain enough distinguishable features for image-based position tracking.
        
        %% 'drivetrainState'
        
        DrivetrainStateNotAvailable = nan; %The state is not available.
        DrivetrainStateNormal = 0; %The drivetrain is operating normally.
        DrivetrainStateUncalibratedCurrent = 1; %The drivetrain is missing current calibration.
        DrivetrainStateUncalibratedPosition = 2; %The drivetrain is missing position calibration.
        DrivetrainStateUncalibratedEffort = 3; %The drivetrain is missing effort calibration.
        
    end
    
end

