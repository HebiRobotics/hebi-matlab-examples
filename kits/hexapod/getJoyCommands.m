function [xyzVel, rotVel, auxCmd] = getJoyCommands( joy )

    % Assumes 'DUALSHOCK®4 USB Wireless Adapto'

    xyzVel = zeros(3,1);
    rotVel = zeros(3,1);
    auxCmd = struct();

    [axes, buttons, povs] = read(joy);

    QUIT_BUTTON = 11;
    STEP_TOGGLE = 12;

    % PRESS DOWN LEFT JOYSTICK TO QUIT
    if buttons(QUIT_BUTTON)
        auxCmd.quit = true;
    else
        auxCmd.quit = false;
    end
    
    % PRESS A TO TOGGLE STEPPING ON AND OFF
    if buttons(STEP_TOGGLE)
        auxCmd.toggleStepping = true;
    else
        auxCmd.toggleStepping = false;
    end
    
    joyXYZScale = .3;
    joyRotScale = .6;
    joyAxisDeadZone = .06;

    % LINEAR VELOCITIES
    % X-Axis
    if abs(axes(6)) > joyAxisDeadZone
        xyzVel(1) = joyXYZScale * axes(6);
    end
    % Y-Axis
    if abs(axes(3)) > joyAxisDeadZone
        xyzVel(2) = joyXYZScale * axes(3);
    end
    % Z-Axis
    if abs(axes(4)-axes(5)) > joyAxisDeadZone
        xyzVel(3) = joyXYZScale * (axes(4)-axes(5));
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
    if abs(axes(1)) > joyAxisDeadZone
        rotVel(3) = joyRotScale * axes(1);
    end

        
end