%% Custom PID - Controller
% This example shows how to use Strategy: DIRECT_PWM to implement
% a custom PID controller that runs in one MATLAB instance and can
% receive targets from another MATLAB instance.
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

%% Select devices to form a target Group
family = '*'; % any family
name = 'Base'; % <- modify to match the name of a device on your network!
group = HebiLookup.newGroupFromNames(family, name); 
disp(group);

%% Create shared memory to communicate with other instance
% This creates a cmdPosition() function that returns a target
% set point that can be modified via shared memory. Note that the 
% dimensions need to be the same for both instances
target = SharedData('target', 'double', [1 group.getNumModules]);
cmdPosition = @() target.data;
disp(cmdPosition());

%% Set control strategy to DIRECT_PWM
gains = GainStruct();
gains.controlStrategy = ones(1,group.getNumModules);
group.send('gains', gains);

%% Simple Proportional (P) Controller on position
% Note: you may want to change the default rate by changing
% group.setFeedbackFrequency(f);
kP = 1;

cmd = CommandStruct();
fbk = group.getNextFeedback();
t0 = tic();
while toc(t0) < 60
    
    % read feedback and target position
    fbk = group.getNextFeedback(fbk);
    cmdPos = cmdPosition();
    
    % proportional control
    error = cmdPos - fbk.position;
    pwm = kP * error;

    % send PWM to robot
    cmd.effort = pwm;
    group.send(cmd);
    
end

%% PID Controller
% Note that this is a bare bones PID controller. For practical
% purposes you'd probably want to add a limit on the I-windup,
% dead-zones, etc.
% [TODO: check that ID controllers use dt correctly]
group.setFeedbackFrequency(1000);
kP = 1;
kI = 0;
kD = 0;

cmd = CommandStruct();
fbk = group.getNextFeedbackFull();
tPrev = fbk.hwTxTime; % use hardware timestamps (note: is a vector)
iError = zeros(size(t0));
lastError = cmdPosition() - fbk.position;
t0 = tic();
while toc(t0) < 60
    
    % read feedback and calculate dt (based on device specific hw timestamps)
    fbk = group.getNextFeedback(fbk);
    t = fbk.hwTxTime;
    dt =  t - tPrev;
    tPrev = t;
    
    % read target and calculate error for this iteration
    error = cmdPosition() - fbk.position;
    
    % proportional control
    pwm = kP .* error;

    % integral control
    iError = iError + error .* dt;
    pwm = pwm + kI .* iError;
    
    % derivative control
    dError = (error - lastError) ./ dt;
    lastError = error;
    pwm = pwm + kD .* dError;
    
    % send PWM to robot
    cmd.effort = pwm;
    group.send(cmd);
    
end
