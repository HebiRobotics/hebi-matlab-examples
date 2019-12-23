function [xyzVel, rotVel, auxCmd] = getMSIJoyCommands( joy )

    % Assumes 'DUALSHOCK®4 USB Wireless Adapto'

    xyzVel = zeros(3,1);
    rotVel = zeros(3,1);
    auxCmd = [];
    
    deadZoneX = [ 2.30 2.80 ];
    deadZoneY = [ 2.00 3.05 ];
    
    limX = [ 0.30 4.70 ];
    limY = [ 0.60 4.50 ];
    
    rangeX = abs(deadZoneX - limX);
    rangeY = abs(deadZoneY - limY);
    
    joyXYZScale = .3;
    joyRotScale = .6;
    joyAxisDeadZone = .06;

    fbk = joy.getNextFeedbackIO;
    xJoyRaw = fbk.a2;
    yJoyRaw = fbk.a1;
    
    % X-Axis Translation
    if yJoyRaw < deadZoneX(1)
        yJoyVal = (yJoyRaw - deadZoneY(1)) / rangeY(1);
    elseif yJoyRaw > deadZoneX(2)
        yJoyVal = (yJoyRaw - deadZoneY(2)) / rangeY(2);
    else
        yJoyVal = 0;
    end
    xyzVel(1) = -joyXYZScale * yJoyVal;
    
    % Z-Axis Rotation
    if xJoyRaw < deadZoneX(1)
        xJoyVal = (xJoyRaw - deadZoneX(1)) / rangeX(1);
    elseif xJoyRaw > deadZoneX(2)
        xJoyVal = (xJoyRaw - deadZoneX(2)) / rangeX(2);
    else
        xJoyVal = 0;
    end
    rotVel(3) = joyRotScale * xJoyVal;

end