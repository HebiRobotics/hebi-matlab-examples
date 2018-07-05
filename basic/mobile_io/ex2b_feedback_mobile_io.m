% Get analog feedback from the touch screen of a mobile device, log in the 
% background, visualize live, and plot offline.
%
% Assumes that you have a group created with at least 1 module in it.
%
% HEBI Robotics
% June 2018

clear *;
close all;

HebiLookup.initialize();

% Use Scope to change select a module and change the name and family to
% match the names below.  Following examples will use the same names.
familyName = 'My Family';
moduleNames = 'Test Mobile';  
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

exampleDuration = 15; % sec
exampleTimer = tic;

group.startLog( 'dir', 'logs' );  % Starts logging in the background

disp('  Drag the sliders and press some buttons on the app screen...');

fbk = group.getNextFeedbackIO();

figure(1);
clf;

while toc(exampleTimer) < exampleDuration
    
   fbk = group.getNextFeedbackIO();
   
   bar( [fbk.a1 fbk.a2 fbk.a3 fbk.a4 fbk.a5 fbk.a6 fbk.a7 fbk.a8] );
   
   yAxisMaxLim = 1; 
   yAxisMinLim = -1;
   ylim([yAxisMinLim yAxisMaxLim]);
   
   title('Analog Inputs');
   ylabel('[-1 to 1]');
   grid on;
   
   drawnow;
   
end

disp('  All done!');

log = group.stopLogIO();  % Stops background logging

% Plot the logged analog feedback
figure(101);
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
legend( strsplit(num2str(1:8)) );
grid on;


