%% I/O Board Pin Mapping
potentiometer = 'a1'; % analog input (0-5 volts)
loadCell      = 'a8'; % analog input (0-5 volts)
inputButton   = 'b1'; % digital input (0 or 1)
led_r         = 'e6'; % digital output (0 or 1)
led_g         = 'e7'; % digital output (0 or 1)
led_b         = 'e8'; % digital output (0 or 1)

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
    ioBoard.send(cmd);

end
