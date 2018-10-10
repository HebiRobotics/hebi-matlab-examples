function [ cmd, cmdIO, abortFlag ] = executeTrajectory( armGroup, ...
                                        phoneGroup, ioGroup, cmd, cmdIO, ...
                                        kin, traj, otherInfo )
%EXECUTETRAJECTORY Plays out a trajectory similar to how the old blocking
%API works.

    % These are copied from the higher level.  Not the best practice :-(
    setX = otherInfo.setX;      % [ticks] to increment
    setY = otherInfo.setY;      % [ticks] to increment
    encoderResX = otherInfo.encoderResX; % [tics / mm] * [mm / m]
    encoderResY = otherInfo.encoderResY; % [tics / mm] * [mm / m]
    probeXYZ_init = otherInfo.probeXYZ_init;
    
    scanSpeed = otherInfo.scanSpeed;  
    downForce = otherInfo.downForce;
    maxPushDownForce = otherInfo.maxPushDownForce;
    startStop = otherInfo.startStop;
    wristAdjust = otherInfo.wristAdjust;
    wristAdjustScale = otherInfo.wristAdjustScale;
    
    damperGains = otherInfo.damperGains;  % [N/(m/sec)] or [Nm/(rad/sec)]
    springGains = otherInfo.springGains;  % [N/m] or [Nm/rad]
    
    numDoF = kin.getNumDoF();

    fbk = armGroup.getNextFeedback();
    t = 0;
    tLast = fbk.time;
    
    abortFlag = false;
    
    % Assume gravity points down in base frame
    gravityVec = [0 0 -1];
    
    pushDownWrench = [0; 0; -maxPushDownForce; 0; 0; 0];
    
    while (t <= traj.getDuration) && (t >= 0) && ~abortFlag

        fbk = armGroup.getNextFeedback();
        dt = fbk.time - tLast;
        tLast = fbk.time;
        
        % Check for M-Stop and throw an error to restart if detected
        fbkIO = armGroup.getNextFeedbackIO();
        if any(fbkIO.a1==0)
            error('M-Stop Detected!');
        end
        
        newPhoneFbkIO = phoneGroup.getNextFeedbackIO( 'timeout', 0 );
        if ~isempty(newPhoneFbkIO)
            phoneFbkIO = newPhoneFbkIO;
        end
        
        if exist('phoneFbkIO','var')
            timeScale = .5 * (1 + phoneFbkIO.(scanSpeed));  % scale [0-1]
            downForceScale = .5 * (1 + phoneFbkIO.(downForce));  % scale [0-1]
            wristPosTweak = phoneFbkIO.(wristAdjust) * wristAdjustScale;
            stopRaster = phoneFbkIO.(startStop);
        else
            timeScale = 1;   
            downForceScale = 1;
            wristPosTweak = 0;
            stopRaster = 0;
        end
        
        if stopRaster==1
            abortFlag = true;
        end
        
        t = t + timeScale*dt;

        % Get commanded positions, velocities, and accelerations
        % from the new trajectory state at the current time.
        [pos, vel, acc] = traj.getState(t);
        % pos = pos;
        vel = timeScale * vel;
        acc = timeScale^2 * acc;
        
        armTipFK = kin.getFK('endeffector',fbk.position);
        armTipFK_cmd = kin.getFK('endeffector',pos);
        J_armTip = kin.getJacobian('endeffector',fbk.position);
        
        % Calculate Impedence Control Wrenches and Appropraite Joint Torque
        springWrench = zeros(6,1);
        damperWrench = zeros(6,1);
        
        % Linear error is easy
        xyzError = armTipFK_cmd(1:3,4) - armTipFK(1:3,4);
        
        % Rotational error involves calculating axis-angle from the
        % resulting error in S03 and providing a torque around that axis.
        errorRotMat = armTipFK_cmd(1:3,1:3) * armTipFK(1:3,1:3)';
        [axis, angle] = HebiUtils.rotMat2axAng( errorRotMat );
        rotErrorVec = angle * axis;
        
%         if gainsInEndEffectorFrame
%             xyzError = armTipFK(1:3,1:3)' * xyzError;
%             rotErrorVec = armTipFK(1:3,1:3)' * rotErrorVec;
%         end
        
        posError = [xyzError; rotErrorVec];
        velError = J_armTip * (vel - fbk.velocity)';     

        springWrench(1:3) = springGains(1:3) .* posError(1:3); % linear force
        springWrench(4:6) = springGains(4:6) .* posError(4:6); % rotational torque
        
%         if gainsInEndEffectorFrame
%             springWrench(1:3) = armTipFK(1:3,1:3) * springWrench(1:3);
%             springWrench(4:6) = armTipFK(1:3,1:3) * springWrench(4:6);
%         end
        
        damperWrench(1:3) = damperGains(1:3) .* velError(1:3); % linear damping
        damperWrench(4:6) = damperGains(4:6) .* velError(4:6); % rotational damping
        
        fullWrench = springWrench + damperWrench + ...
                                        downForceScale*pushDownWrench;
        impedanceEffort = J_armTip' * fullWrench; 
        
        % Compensate for gravity
        gravCompEffort = kin.getGravCompEfforts( ...
                                    fbk.position, gravityVec );

        % Compensate for dynamics based on the new commands
        accelCompEffort = kin.getDynamicCompEfforts(...
            fbk.position, ... % Used for calculating jacobian
            pos, vel, acc);
        
        % Tweak the wrist a little bit
        pos(4) = pos(4) + wristPosTweak;
        
        armGroup.send(cmd);

        cmd.position = nan(1,numDoF);
        cmd.position = pos;
        cmd.velocity = vel;
        cmd.effort = gravCompEffort + accelCompEffort + impedanceEffort';
 
        % Get probe position from FK
        probeXYZ = armTipFK(1:3,4) - probeXYZ_init;

        cmdIO.(setX) = round(encoderResX * probeXYZ(1));
        cmdIO.(setY) = round(encoderResY * probeXYZ(2));
        ioGroup.send(cmdIO);
    end
end

