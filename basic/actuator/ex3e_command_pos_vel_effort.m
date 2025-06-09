% Send simultaneous position/velocity/effort commands, log in the 
% background, and plot offline.
%
% For more information type:
%    help CommandStruct
%    help HebiGroup
%
% This script assumes you can create a group with 1 module.  It also
% assumes that a load with some mass / inertia is attached to the output. 
% If the output is unloaded, these commands will track worse than the
% previous example that only commanded position + velocity.  See the
% comments about inertia above Line 40 below.
%
% HEBI Robotics
% June 2018

%% Setup
clear *;
close all;
HebiLookup.initialize();

familyName = 'Test Family';
moduleNames = 'Test Actuator'; 
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

%% Open-Loop Controller (Position + Velocity + Effort)
cmd = CommandStruct();
group.startLog( 'dir', 'logs' );

% Parameters for sin/cos function
freqHz = 1.0;           % [Hz]
freq = freqHz * 2*pi;   % [rad / sec]
amp = deg2rad( 45 );    % [rad]

% Inertia parameters for converting acceleration to torque.  This inertia
% value corresponds to roughly a 300mm X5 link extending off the output. 
inertia = .01; % [kg * m^2]

duration = 10; % [sec]
timer = tic();
while toc(timer) < duration
    
    % Even though we don't use the feedback, getting feedback conveniently 
    % limits the loop rate to the feedback frequency 
    fbk = group.getNextFeedback();
    t = toc(timer);

    % Position command
    cmdPosition = amp * sin( freq * t );

    % Velocity command (time-derivative of position)
    cmdVelocity = freq * amp * cos( freq * t );

    % Acceleration command (time-derivative of velocity)
    cmdAcceleration = -freq^2 * amp * sin( freq * t );

    % Update set points
    cmd.position = cmdPosition;
    cmd.velocity = cmdVelocity;
    cmd.effort = inertia * cmdAcceleration;
    group.send(cmd);
   
end

% Stop logging and plot the commands using helper functions
log = group.stopLog();
HebiUtils.plotLogs( log, 'position', 'figNum', 101 );
HebiUtils.plotLogs( log, 'velocity', 'figNum', 102 );
HebiUtils.plotLogs( log, 'effort', 'figNum', 103 );
