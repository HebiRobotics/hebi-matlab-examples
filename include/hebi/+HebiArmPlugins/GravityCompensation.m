classdef GravityCompensation < HebiArmPlugin
    % GravityCompensation arm plugin
    %
    % Adds efforts to make the arm appear weightless. Note that
    % the arm may not remain still due to imperfect feedback or 
    % modeling errors.
    
    properties
        % Zero-based index of the device within the feedback group (default: first)
        imuFeedbackIndex uint32 = 0;

        % Zero-based index of the frame of the imu feedback (default: first)
        imuFrameIndex uint32 = 0;

        % Rotation applied to the IMU feedback. Defaults to eye(3)
        imuRotationOffset double = eye(3);
    end
    
    methods
        
        function this = GravityCompensation()
        end
        
        function [] = update(this, arm)

            % Find the orientation in the imu frame
            fbk = arm.state.fbk;
            q = [ fbk.orientationW(this.imuFeedbackIndex + 1), ...
                  fbk.orientationX(this.imuFeedbackIndex + 1), ...
                  fbk.orientationY(this.imuFeedbackIndex + 1), ...
                  fbk.orientationZ(this.imuFeedbackIndex + 1) ];

            % Transform gravity vector into the world frame
            if any(isnan(q))
                % If the group does not provide orientation feedback, we assume
                % that gravity points 'down' in the base frame (-Z Axis).
                warning('No orientation feedback available. Assuming gravity points down.');
                gravityVec = [0; 0; -1];
            else
                % The orientation feedback is in the IMU frame, so we 
                % need to first rotate it into the world frame.
                imuRot = HebiUtils.quat2rotMat(q);
                imuGravityVec = -imuRot(3,1:3)';
                
                baseGravityVec = this.imuRotationOffset * imuGravityVec;

                % The imu frame is at the input of the specified index
                % (zero-based). A value of zero means the input of body{1},
                % i.e., the output of the base frame. Higher values are off
                % by 1 (assuming a serial chain), which works out with
                % MATLAB's one-based indexing.
                if this.imuFrameIndex == 0
                    imuFrame = arm.kin.getBaseFrame();
                else
                    imuFrame = arm.state.outputFrames(:,:,this.imuFrameIndex);
                end

                gravityVec = imuFrame(1:3,1:3) * baseGravityVec;
            end

            % Compensate for gravity
            gravCompEfforts = arm.kin.getGravCompEfforts(fbk.position, gravityVec);

            % Add to efforts
            rampScale = this.getRampScale(arm.state.time);
            arm.state.cmdEffort = arm.state.cmdEffort + gravCompEfforts .* rampScale;
            
        end
        
    end
    
end
