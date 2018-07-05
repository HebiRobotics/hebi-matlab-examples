% Get feedback from a module, log in the background, and plot offline.
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
moduleNames = 'Test I/O Board';  
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

exampleDuration = 10; % sec
exampleTimer = tic;

group.startLog( 'dir', 'logs' );  % Starts logging in the background

disp('  We need to come up with something fun for the I/O Board...');

fbk = group.getNextFeedbackIO();

figure(1);
clf;

while toc(exampleTimer) < exampleDuration
    
   fbk = group.getNextFeedbackIO();
   
   bar( fbk.a1 );
   
   yAxisMaxLim = 5; 
   yAxisMinLim = -5;
   ylim([yAxisMinLim yAxisMaxLim]);
   
   title( 'Module Output Velocity' );
   ylabel( 'Angular Velocity (rad/sec)');
   grid on;
   
   drawnow;
   
end

disp('  All done!');

log = group.stopLog();  % Stops background logging

% Plot the logged position feedback
figure(101);
plot(log.time,log.a1);
title('Position');
xlabel('time (sec)');
ylabel('position (rad)');
grid on;

% Plot the logged velocity feedback
figure(102);
plot(log.time,log.d1);
title('Velocity');
xlabel('time (sec)');
ylabel('velocity (rad/sec)');
grid on;


