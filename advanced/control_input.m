%% Setup
group = HebiLookup.newGroupFromNames('Family','Name');
group.setFeedbackFrequency(100);

%% Step input
amplitude = 0.5; % [rad]
interval = 1.5; % [s]
runtime = 100; % [s]

cmd = CommandStruct();
t0 = tic();
group.startLog('dir', 'logs');
while toc(t0) < runtime
    
    direction = 1;
    if mod(toc(t0),interval*2) < interval
        direction = -1;
    end
    
    cmd.position = amplitude * direction;
    group.send(cmd);
    pause(0.001);
    
end

hLog = group.stopLogFull();
HebiUtils.plotLogs(hLog,'position');

%% Sinusoidal Input
A = 1;
f = 0.5; % [Hz]
w = 2*pi*f;
runtime = 30; % [s]

cmd = CommandStruct();
t0 = tic();
group.startLog('dir', 'logs');
while toc(t0) < runtime
    
    t = toc(t0);    
    cmd.position = A * sin(w*t);
    cmd.velocity = A * w * cos(w*t);
%     accel = A * w^2 * -sin(w*t);
    group.send(cmd);
    pause(0.001);
    
end

hLog = group.stopLogFull();
HebiUtils.plotLogs(hLog,'position');

%% Sinusoidal w/ feed forward torques
kin = HebiKinematics();
kin.addBody('X5-4');
kin.addBody('X5-Link', 'extension', 0.250, 'twist', pi);
% kin.setPayload(0.360, 'com', [0 0 0]);

% Determine gravity vector (assumes fixed base)
fbk = group.getNextFeedback();
gravityVec = -[fbk.accelX(1) fbk.accelY(1) fbk.accelZ(1)];

A = 1;
f = 0.5; % [Hz]
w = 2*pi*f;
runtime = 30; % [s]

cmd = CommandStruct();
t0 = tic();
group.startLog('dir', 'logs');
while toc(t0) < runtime
    
    t = toc(t0);    
    p = A * sin(w*t);
    v = A * w * cos(w*t);
    a = A * w^2 * -sin(w*t);
    
    cmd.position = p;
    cmd.velocity = v;
    cmd.effort = ...
        + kin.getGravCompEfforts(p, gravityVec) ...
        + kin.getDynamicCompEfforts(p,p,v,a);
    
    group.send(cmd);
    pause(0.001);
    
end

hLog = group.stopLogFull();
HebiUtils.plotLogs(hLog,'position');

%% Trajectories
kin = HebiKinematics();
kin.addBody('X5-4');
kin.addBody('X5-Link', 'extension', 0.250, 'twist', pi);

% Determine gravity vector (assumes fixed base)
fbk = group.getNextFeedback();
gravityVec = -[fbk.accelX(1) fbk.accelY(1) fbk.accelZ(1)];

trajGen = HebiTrajectoryGenerator(kin);
trajGen.setMinDuration(0.2);
trajGen.setSpeedFactor(0.5);

amplitude = 0.5; % [rad]
interval = 1.5; % [s]
runtime = 100; % [s]

cmd = CommandStruct();
t0 = tic();
group.startLog('dir', 'logs');
traj = trajGen.newJointMove(direction .* [amplitude -amplitude]);
while toc(t0) < runtime
    
    t = toc(t0);
    if t > interval
       direction = direction * -1;
       traj = trajGen.newJointMove(direction .* [amplitude -amplitude]);
       t0 = tic();
    end
    
    t = min(t,traj.getDuration);
    [p,v,a] = traj.getState(t);
    
    cmd.position = p;
    cmd.velocity = v;
    cmd.effort = ...
        + kin.getGravCompEfforts(p, gravityVec) ...
        + kin.getDynamicCompEfforts(p,p,v,a);
    
    group.send(cmd);
    pause(0.001);
    
end

hLog = group.stopLogFull();
HebiUtils.plotLogs(hLog,'position');


