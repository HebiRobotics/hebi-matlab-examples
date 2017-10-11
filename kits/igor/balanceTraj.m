% Test out balancing velocity trajectories
% 
% Dave Rollinson
% Mar 2017

% Setup kinematics and create trajectory generator
kin = HebiKinematics();
kin.addBody('X5-4');

velNow = 0;
accNow = 0;
jerkNow = 0;

maxAccel = 4;

joy = HebiJoystick(1);

while true
    
    [axes, buttons, povs] = read(joy);
    
    QUIT_BUTTON = 11;
    if buttons(QUIT_BUTTON)
        break;
    end
    
    cmdVel = 5*axes(6);
    rampTime = max(abs(cmdVel-velNow)/maxAccel,.25);

    time = [ 0 rampTime ];
    vels = [velNow cmdVel];
    accels = [accNow 0 ];
    jerks = [jerkNow 0];

    trajGen = HebiTrajectoryGenerator(kin);
    trajGen.setSpeedFactor(1);
    trajGen.setAlgorithm('UnconstrainedQp');

    trajectory = trajGen.newJointMove( vels, ...
                'Velocities', accels, ...
                'Accelerations', jerks, ...
                'Time', time );
    tic;
    
    % Visualize trajectory
    t = 0:0.01:trajectory.getDuration();
    [vel, accel, jerk] = trajectory.getState(t);
    figure(101);
    plot(t, vel);
    hold on;
    plot(t, accel);
    %plot(t, jerk);
    hold off;
    ylim([-10,10]);
    drawnow;

    pause(.1);
    
    t = toc;
    [velNow, accNow, jerkNow] = trajectory.getState(t);
    

end
