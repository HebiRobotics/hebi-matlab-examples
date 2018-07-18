% Get analog feedback from the touch screen of a mobile device, log in the
% background, visualize live, and plot offline.
%
% Assumes that you have a group created with at least 1 module in it.
%
% HEBI Robotics
% July 2018

%% Setup
clear *;
close all;
HebiLookup.initialize();

% Use Scope to change select a module and change the name and family to
% match the names below.  Following examples will use the same names.
familyName = 'HEBI';
moduleNames = 'Virtual IO';
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

%% Visualize Slider Input
disp('  Drag the sliders and press some buttons on the app screen...');

% Start logging in the background
group.startLog( 'dir', 'logs' ); 

figure(1);
clf;

duration = 15; % [sec]
timer = tic();
while toc(timer) < duration
    
    % Read feedback
    fbk = group.getNextFeedbackIO();
    
    % Digital feedback
    bar( [fbk.b1 fbk.b2 fbk.b3 fbk.b4 fbk.b5 fbk.b6 fbk.b7 fbk.b8], 'r' );
    hold on;
    
    % Analog feedback
    bar( [fbk.a1 fbk.a2 fbk.a3 fbk.a4 fbk.a5 fbk.a6 fbk.a7 fbk.a8], 'b' );
    hold off;
    
    % Format plot
    yAxisMaxLim = 1;
    yAxisMinLim = -1;
    ylim([yAxisMinLim yAxisMaxLim]);
    title('Digital Inputs (red) and Analog Inputs (blue)');
    ylabel('[-1 to 1]');
    grid on;
    drawnow;
    
end

disp('  All done!');

% Stop background logging
log = group.stopLogIO();  

%% Plot the logged feedback
figure(101);
subplot(2,1,1);
plot(log.time,log.a1);
hold on;
plot(log.time,log.a2);
plot(log.time,log.a3);
plot(log.time,log.a4);
plot(log.time,log.a5);
plot(log.time,log.a6);
plot(log.time,log.a7);
plot(log.time,log.a8);
hold off;

title('Analog Inputs');
xlabel('time (sec)');
ylabel('[-1 to 1]');
ylim([-1.1 1.1]);
legend( strsplit(num2str(1:8)) );
grid on;

subplot(2,1,2);
plot(log.time,log.b1,'.');
hold on;
plot(log.time,log.b2,'.');
plot(log.time,log.b3,'.');
plot(log.time,log.b4,'.');
plot(log.time,log.b5,'.');
plot(log.time,log.b6,'.');
plot(log.time,log.b7,'.');
plot(log.time,log.b8,'.');
hold off;

title('Digital Inputs');
xlabel('time (sec)');
ylabel('[0 or 1]');
ylim([-0.1 1.1]);
legend( strsplit(num2str(1:8)) );
grid on;

