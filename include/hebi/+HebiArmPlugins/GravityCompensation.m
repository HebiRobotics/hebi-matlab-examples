classdef GravityCompensation < HebiArmPlugin
    % GravityCompensation arm plugin
    %
    % Adds efforts to make the arm appear weightless. Note that
    % the arm may not remain still due to imperfect feedback or 
    % modeling errors.
    
    properties
        % Index of the device within the feedback group (default: first)
        imuFeedbackIndex uint32 = 1;

        % Index of the frame of the imu feedback (default: first)
        imuFrameIndex uint32 = 1;

        % Rotation applied to the IMU feedback. Defaults to eye(3)
        imuRotationOffset double = eye(3);
    end
    
    methods
        
        function this = GravityCompensation()
        end
        
        function [] = update(this, arm)

            % Find the orientation in the imu frame
            fbk = arm.state.fbk;
            q = [ fbk.orientationW(this.imuFeedbackIndex), ...
                  fbk.orientationX(this.imuFeedbackIndex), ...
                  fbk.orientationY(this.imuFeedbackIndex), ...
                  fbk.orientationZ(this.imuFeedbackIndex) ];

            % Transform gravity vector into the world frame
            if any(isnan(q))
                % If the group does not provide orientation feedback, we assume
                % that gravity points 'down' in the base frame (-Z Axis).
                warning('No orientation feedback available. Assuming gravity points down.');
                gravityVec = [0; 0; -1];
            else
                % The orientation feedback is in the IMU frame, so we 
                % need to first rotate it into the world frame.
                baseRotMat = this.imuRotationOffset * HebiUtils.quat2rotMat(q);
                imuFrame = arm.state.outputFrames(:,:,this.imuFrameIndex);
                gravityVec = imuFrame(1:3,1:3) * (-baseRotMat(3,1:3)');
            end

            % Compensate for gravity
            gravCompEfforts = arm.kin.getGravCompEfforts(fbk.position, gravityVec);

            % Add to efforts
            rampScale = this.getRampScale(arm.state.time);
            arm.state.cmdEffort = arm.state.cmdEffort + gravCompEfforts .* rampScale;
            
        end
        
    end
    
end
