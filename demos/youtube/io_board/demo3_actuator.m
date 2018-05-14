%% I/O Board Pin Mapping
potentiometer = 'b1';  % analog input (0-5 volts)

%% Setup Groups
ioBoard = HebiLookup.newGroupFromNames('*', 'IO_basic');
actuator = HebiLookup.newGroupFromNames('*', 'X5-1_00004');

%% Command Actuator Position based on Ananlog Input
tmpFbk = ioBoard.getNextFeedbackIO();
cmd = CommandStruct();

positionCmdScale = 2*pi;  % Scales voltage to radians

while true
    
    % Map analog sensor feedback to position command
    fbk = ioBoard.getNextFeedback(tmpFbk);
    cmd.position = fbk.(potentiometer) * positionCmdScale;
    actuator.set(cmd);

end
