% Display joystick data 
%
% Dave Rollinson
% Feb 2017

HebiJoystick.loadLibs();
joy = HebiJoystick(1)

disp('Displaying button states, ctrl-C to quit...');

figure(345);
while true
    
    [axes, buttons, povs] = read(joy);
    
    subplot(3,1,1);
    plot(1:length(axes),axes,'o');
    title('Axes');
    ylim([-1 1]);
    xlim([.5 length(axes)+.5]);
    set(gca,'xTick',1:length(axes));
    grid on;
    
    subplot(3,1,2);
    plot(1:length(buttons),buttons,'*');
    title('Buttons');
    ylim([0 1]);
    xlim([.5 length(buttons)+.5]);
    set(gca,'xTick',1:length(buttons));
    grid on;
    
    subplot(3,1,3);
    if povs == -1   
        plot(0,0,'o');
    else
        plot([0 sin(deg2rad(povs))],[0 cos(deg2rad(povs))]);
    end
    title('POVs');
    text(.5,.75,['Val: ' num2str(povs)])
    axis equal;
    ylim([-1 1]);
    xlim([-1 1]);
    grid on;
    
    drawnow;
end