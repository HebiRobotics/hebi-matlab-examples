%% I/O Box Pin Mapping
potentiometer = 'b1'; % analog input  (0-5 volts)
led_r = 'c6';         % digital input (0 or 1)

%% Setup Groups
ioBoard = HebiLookup.newGroupFromNames('*', 'IO_basic');

%% Start Background Logging
ioBoard.startLog();

%% Flash red LED while data is recording
cmd = IoCommandStruct();
cmd.(led_r) = 1;
t = tic();

blinkPeriod = 0.1;

while toc(t) < 5
    cmd.(led_r) = ~cmd.(led_r);
    ioBoard.set(cmd);
    pause(blinkPeriod); 
end

cmd.(led_r) = 0;
ioBoard.set(cmd);

%% Stop Logging and Retreive Data
log = ioBoard.stopLogIO();

%% Visualize Logged Data
figure(101);
hold off;
plot(log.time, log.(potentiometer), 'LineWidth', 2);
xlim( [0 log.time(end)] );
ylim( [0 1] );
xlabel('time (s)');
ylabel('potentiometer (V)');
