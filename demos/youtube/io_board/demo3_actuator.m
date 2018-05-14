%% I/O Board Pin Mapping
potentiometer = 'b1';  % analog input (0-5 volts)

%% Setup Groups
ioBoard = HebiLookup.newGroupFromNames('*', 'IO_basic');
actuator = HebiLookup.newGroupFromNames('*', 'X5-1_00004');

%% Command Actuator Position based on Ananlog Input
tmpFbk = ioBoard.getNextFeedbackIO();
cmd = CommandStruct();

while true
    
    % Map analog sensor feedback to position command
    fbk = ioBoard.getNextFeedback(tmpFbk);
    cmd.position = fbk.(potentiometer) * 2*pi;
    actuator.set(cmd);

end
