% Get feedback from a module, log in the background, and plot offline.
%
% Assumes that you have a group created with at least 1 module in it.
%
% HEBI Robotics
% Jun 2018

HebiLookup.initialize();

% Use Scope to change select a module and change the name and family to
% match the names below.  Following examples will use the same names.
familyName = 'My Family';
moduleNames = 'Test Module';  
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

exampleDuration = 20; % sec

tic;

group.startLog();  % Starts logging in the background

disp('  Use Scope to command the module and make it move...');

while toc < exampleDuration
    
   fbk = group.getNextFeedback();
   
   plot(fbk.position,'.');
   drawnow;
   
end

disp('  All done!');

log = group.stopLog();  % Stops background logging

% Plot the logged position feedback
figure(1);
plot(log.time,log.position);
title('Position');
xlabel('time (sec)');
ylabel('position (rad)');

% Plot the logged velocity feedback
figure(2);
plot(log.time,log.velocity);
title('Velocity');
xlabel('time (sec)');
ylabel('velocity (rad/sec)');
