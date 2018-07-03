% Get feedback from a module and plot it live.
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

exampleDuration = 10; % sec

exampleTimer = tic;

disp('  Plotting gyro data from the module(s) IMU.');
disp('  Move the module around to make the feedback interesting...');  

while toc(exampleTimer) < exampleDuration
    
   fbk = group.getNextFeedback();
   
   plot(fbk.gyroX,'.');
   hold on;
   plot(fbk.gyroY,'*');
   plot(fbk.gyroZ,'o');
   hold off;
   drawnow;
   
end

disp('  All Done!');
