% Get feedback from a module and plot it live.
%
% Assumes that you have a group created with at least 1 module in it.
%
% HEBI Robotics
% Jun 2018

HebiLookup.initialize();

% Do we assume that they just keep using the group they created earlier?
% Or do we want to make a new group each time?

exampleDuration = 10; % sec

tic;

disp('  Plotting gyro data from the module(s) IMU.');
disp('  Move the module around to make the feedback interesting...');  

while toc < exampleDuration
    
   fbk = group.getNextFeedback();
   
   plot(fbk.gyroX,'.');
   hold on;
   plot(fbk.gyroY,'*');
   plot(fbk.gyroZ,'o');
   hold off;
   drawnow;
   
end

disp('  All Done!');
