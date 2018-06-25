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

while toc < exampleDuration
    
   fbk = group.getNextFeedback();
   
   % While this loop runs, pick the module up and move it around
   
   plot(fbk.gyroX,'.');
   hold on;
   plot(fbk.gyroY,'*');
   plot(fbk.gyroZ,'o');
   hold off;
   drawnow;
   
end


