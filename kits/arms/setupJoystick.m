function [ joy, joyNames ] = setupJoystick()

    joy = HebiJoystick(1);
    pause(0.1);

    joyNames.LEFT_STICK_X = 1;
    joyNames.LEFT_STICK_Y = 2;
    joyNames.OPTIONS_BUTTON = 10;
    joyNames.LEFT_TRIGGER_BUTTON = 5;
    joyNames.RIGHT_TRIGGER_BUTTON = 6;
    joyNames.SHARE_BUTTON = 9;

    
    if isunix && ~ismac
        % Linux Mappings
        joyNames.SQUARE_BUTTON = 4;
        joyNames.CIRCLE_BUTTON = 2;
        joyNames.TRIANGLE_BUTTON = 3;
        joyNames.X_BUTTON = 1;
        joyNames.LEFT_STICK_CLICK = 12;
        joyNames.TOUCHPAD_BUTTON = 11;
        joyNames.LEFT_TRIGGER = 3;
        joyNames.RIGHT_TRIGGER = 6;
        joyNames.RIGHT_STICK_X = 4;
        joyNames.RIGHT_STICK_Y = 5;
    else
        % macOS and Windows Mappings
        joyNames.SQUARE_BUTTON = 1;
        joyNames.CIRCLE_BUTTON = 3;
        joyNames.TRIANGLE_BUTTON = 4;
        joyNames.X_BUTTON = 2;
        joyNames.LEFT_STICK_CLICK = 11;
        joyNames.RIGHT_STICK_CLICK = 12;
        joyNames.TOUCHPAD_BUTTON = 14;
        joyNames.LEFT_TRIGGER = 4;
        joyNames.RIGHT_TRIGGER = 5;
        joyNames.RIGHT_STICK_X = 3;
        joyNames.RIGHT_STICK_Y = 6;
    end
end