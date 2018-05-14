%% Setup Figure Handles
lineWidth = 2;
figure(101); 

hold off;
handle1 = plot(1, nan, 'ro', 'LineWidth', lineWidth); 
hold on;
handle2 = plot(2, nan, 'go', 'LineWidth', lineWidth);
handle3 = plot(3, nan, 'bo', 'LineWidth', lineWidth);
hold off;

ylim([-0.1 1.1]); 
xlim([0.9 3.1]); 
ylabel('value [0-1]');

legend button potentiometer loadCell;

%% I/O Board Pin Mapping
potentiometer = 'b1'; % analog input (0-5 volts)
loadCell      = 'b8'; % analog input (0-5 volts)
inputButton   = 'e1'; % digital input (0 or 1)

%% Setup Groups
ioBoard = HebiLookup.newGroupFromNames('*', 'IO_basic');

%% Live Display of Input Values
while true

    % Read feedback
    fbk = ioBoard.getNextFeedbackIO();

    % Update values
    handle1.YData = fbk.(inputButton);
    handle2.YData = fbk.(potentiometer);
    handle3.YData = fbk.(loadCell);

    % Force re-draw
    drawnow;

end