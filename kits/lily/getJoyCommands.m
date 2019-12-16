function [xyzVel, rotVel, auxCmd] = getJoyCommands( fbkIO )

    % Assumes you're using the HEBI Mobile I/O App

    xyzVel = zeros(3,1);
    rotVel = zeros(3,1);
    auxCmd = struct();

    QUIT_BUTTON = 'b8';
    STANCE_MODE = 'b1';

    % PRESS DOWN B8 TO QUIT
    if fbkIO.(QUIT_BUTTON)
        auxCmd.quit = true;
    else
        auxCmd.quit = false;
    end
    
    % PRESS A TO TOGGLE STEPPING ON AND OFF
    if fbkIO.(STANCE_MODE)
        auxCmd.steppingMode = false;    
    else
        auxCmd.steppingMode = true;
    end
    
    joyXYZScale = .3;
    joyRotScale = .6;
    joyAxisDeadZone = .05;

    % LINEAR VELOCITIES
    % X-Axis
    if abs(fbkIO.a8) > joyAxisDeadZone
        xyzVel(1) = -joyXYZScale * fbkIO.a8;
    end
    % Y-Axis
    if abs(fbkIO.a7) > joyAxisDeadZone
        xyzVel(2) = joyXYZScale * fbkIO.a7;
    end
    % Z-Axis
    if abs(fbkIO.a3) > joyAxisDeadZone
        xyzVel(3) = -joyXYZScale * fbkIO.a3;
    end
    
    % Make sure max velocity magnitude is maintained (L2-Norm)
    if norm(xyzVel) > joyXYZScale
        xyzVel = xyzVel * (joyXYZScale/norm(xyzVel));
    end
    
    % ROTATIONAL VELOCITIES
    % X-Axis Rotation
    
%     % Y-Axis Rotation
%     if abs(axes(2)) > joyAxisDeadZone
%         rotVel(2) = joyRotScale * axes(2);
%     end
    
    % Z-Axis Rotation
    if abs(fbkIO.a1) > joyAxisDeadZone
        rotVel(3) = joyRotScale * fbkIO.a1;
    end

        
end