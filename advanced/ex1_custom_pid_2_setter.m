%% Custom PID - Target Setter
% This example shows how the target in pid_1_controller.m can be set
% from another instance. Needs to be run in a separate MATLAB instance.
%
%%
%
% <html>
% <table border=0>
%   <tr><td>Created</td><td>Sept21, 2017</td></tr>
%   <tr><td>Last Update</td><td>Sept 21, 2017</td></tr>
%   <tr><td>API Version</td><td>hebi-matlab-1.0</td></tr>
%   <tr><td>Requirements</td><td>MATLAB 2013b or higher</td></tr>
% </table>
% </html>
%
% Copyright 2017 HEBI Robotics

%% Create shared memory to communicate with other instance
% This creates a cmdPosition() function that returns a target
% set point that can be modified via shared memory. Note that the 
% dimensions need to be the same for both instances
numModules = 1; % MODIFY TO MATCH CONTROLLER !
target = SharedData('target', 'double', [1 numModules]);

%% Step Input (open-loop)
% This example shows an open-loop controller commanding alternating
% step inputs of +/- amplitude to position
amplitude = 1 * ones(1,numModules); % [rad]
duration = 2; % [s]

cmd = CommandStruct();
direction = 1;
tNext = duration;
t0 = tic();
while toc(t0) < 20
    
    % Flip direction
    if toc(t0) >= tNext
        tNext = tNext + duration;
        direction = -direction;
    end
    
    % Command position via shared memory
    target.data = amplitude * direction;
    pause(0.01);
    
end

