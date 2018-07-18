% Put everything together to control a 3-DoF arm.
%
% For more information type:
%    help hebi
%
% HEBI Robotics
% July 2018

%%
clear *;
close all;

HebiLookup.initialize();

kin = HebiKinematics('hrdf/3-DoF_arm_example.hrdf');
gains = HebiUtils.loadGains('gains/3-DoF_arm_gains.xml');
trajGen = HebiTrajectoryGenerator();

% Assume gravity points down in the frame of the first module.  This will
% get used for gravity compensation when controlling the arm.  You can use
% the IMUs in the module to measure the actual orientation of the arm. See
% the advanced examples on how to do this.
gravityVec = [0 0 -1];

familyName = 'My Arm';
moduleNames = {'Base','Shoulder','Elbow'};

group = HebiLookup.newGroupFromNames( familyName, moduleNames );
group.send('gains',gains);
cmd = CommandStruct();

group.startLog('dir','logs');

% Four Corners of a Box
xyzTargets = [ 0.20  0.40  0.40  0.20;    % x [m]
               0.30  0.30 -0.30 -0.30;    % y [m]
               0.10  0.10  0.10  0.10 ];  % z [m]

initPosition = [ 0 -pi/4 -pi/2 ];

% Do IK to get joint position waypoints for each XYZ target.  Copy the
% first waypoint at the end, so that it closes the loop
for i=1:length(xyzTargets)
    posTargets(i,:) = kin.getIK( 'xyz', xyzTargets(:,i), ...
                                'initial', initPosition );                        
end
posTargets(end+1,:) = posTargets(1,:);

% Get the initial feedback joint positions, and go from there to the first
% position target, using the trajectory API.  
fbk = group.getNextFeedback();
waypoints = [ fbk.position;
              posTargets(1,:) ];    % [rad]
timeToMove = 5;              % [sec]
time = [ 0 timeToMove ];     % [sec]

% Calculate initial trajectory to starting posiiton
trajectory = trajGen.newJointMove( waypoints, 'time', time );

% Initialize timer
t0 = fbk.time;
t = 0;

% Execute the motion to go to the first target
while t < trajectory.getDuration
    
    % Get feedback and update the timer
    fbk = group.getNextFeedback();
    t = fbk.time - t0;
    
    % Get new commands from the trajectory
    [pos,vel,acc] = trajectory.getState(t);
    
    % Calculate commanded efforts to assist with tracking the trajectory.
    % gravCompEfforts() uses knowledge of the arm's kinematics and mass to
    % compensate for the weight of the arm.  dynamicCompEfforts() uses the
    % kinematics and mass to compensate for the commanded accelerations of
    % the arm.
    gravCompEfforts = kin.getGravCompEfforts( fbk.position, gravityVec );
    dynamicCompEfforts = kin.getDynamicCompEfforts( fbk.position, ...
                                                    pos, vel, acc );
    
    % Fill in the CommandStruct and send commands to the arm
    cmd.position = pos;
    cmd.velocity = vel;
    cmd.effort = gravCompEfforts + dynamicCompEfforts; 
    
    group.send(cmd);
end

% Go to the other points a little bit more quickly.  
timeToMove = 3;             % [sec]
time = [ 0 timeToMove ];    % [sec]
    
% Go to all the different points.  Calculate new point-to-point
% trajectories one at a time.
for i=1:length(xyzTargets)
    
    % Shift to the next target position
    waypoints = [ posTargets(i,:) ;
                  posTargets(i+1,:) ];

    % Get the trajectory to the next target 
    trajectory = trajGen.newJointMove( waypoints, 'time', time );
    
    % Get feedback and update the timer
    t0 = fbk.time;
    t = 0;
    
    % Execute the motion to go to the next target
    while t < trajectory.getDuration

        % Get feedback and update the timer
        fbk = group.getNextFeedback();
        t = fbk.time - t0;

        % Get new commands from the trajectory
        [pos,vel,acc] = trajectory.getState(t);

        % Calculate commanded efforts to assist with tracking the trajectory.
        % gravCompEfforts() uses knowledge of the arm's kinematics and mass to
        % compensate for the weight of the arm.  dynamicCompEfforts() uses the
        % kinematics and mass to compensate for the commanded accelerations of
        % the arm.
        gravCompEfforts = kin.getGravCompEfforts( fbk.position, gravityVec );
        dynamicCompEfforts = kin.getDynamicCompEfforts( fbk.position, ...
                                                        pos, vel, acc );

        % Fill in the CommandStruct and send commands to the arm
        cmd.position = pos;
        cmd.velocity = vel;
        cmd.effort = gravCompEfforts + dynamicCompEfforts;
        
        group.send(cmd);
    end
    
end

% Stop logging and plot the command vs feedback pos/vel/effort
log = group.stopLog();

HebiUtils.plotLogs( log, 'position', 'figNum', 101 );
HebiUtils.plotLogs( log, 'velocity', 'figNum', 102 );
HebiUtils.plotLogs( log, 'effort', 'figNum', 103 );

