%% I/O Board Pin Mapping
potentiometer = 'b1'; % analog input (0-5 volts)
loadCell      = 'b8'; % analog input (0-5 volts)
inputButton   = 'e1'; % digital input (0 or 1)
led_r         = 'c6'; % digital output (0 or 1)
led_g         = 'c7'; % digital output (0 or 1)
led_b         = 'c8'; % digital output (0 or 1)

%% Setup Groups
ioBoard = HebiLookup.newGroupFromNames('*', 'IO_basic');

%% Live LED Commands
cmd = IOCommandStruct();

loadCellVoltageThreshold = 0.2;
potentiometerVoltageThreshold = 0.5;

while true

    % Read feedback
    fbk = ioBoard.getNextFeedbackIO();

    % Update values
    cmd.(led_r) = fbk.(inputButton);
    cmd.(led_g) = fbk.(loadCell) > loadCellVoltageThreshold;
    cmd.(led_b) = fbk.(potentiometer) > potentiometerVoltageThreshold;

    % Send commands
    ioBoard.set(cmd);

end