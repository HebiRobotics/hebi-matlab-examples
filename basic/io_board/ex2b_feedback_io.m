% Get feedback from a module, log in the background, and plot offline.
%
% Assumes that you have a group created with at least 1 module in it.
%
% HEBI Robotics
% June 2018

%% Setup
clear *;
close all;
HebiLookup.initialize();

% Use Scope to change select a module and change the name and family to
% match the names below.  Following examples will use the same names.
familyName = 'valve_test_board';
moduleNames = 'io_micro'; 
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

%% Visualize analog input a1
disp('  We need to come up with something fun for the I/O Board...');

% Start logging in the background
group.startLog( 'dir', 'logs' );

figure(1);
clf;

duration = 10; % [sec]
timer = tic();
while toc(timer) < duration
    
   % Read feedback
   fbk = group.getNextFeedbackIO();

   % Display
   bar( fbk.a1 );
   
   % Format plot
   yAxisMaxLim = 5; 
   yAxisMinLim = -5;
   ylim([yAxisMinLim yAxisMaxLim]);
   title( 'Module Output Velocity' );
   ylabel( 'Angular Velocity (rad/sec)');
   grid on;
   drawnow;
   
end

disp('  All done!');

% Stop background logging
log = group.stopLogIO();

%% Plot the logged feedback
figure(101);
plot(log.time,log.a1);
title('Pin a1');
xlabel('time (sec)');
ylabel('value');
grid on;
