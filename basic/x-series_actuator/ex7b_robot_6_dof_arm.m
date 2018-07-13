% Put everything together to control a 3-DoF arm.
%
% For more information type:
%    help hebi
%
% HEBI Robotics
% July 2018

clear *;
close all;

HebiLookup.initialize();

kin = HebiKinematics('/hrdf/6-DoF_arm_example.hrdf');
gains = HebiUtils.loadGains('/gains/6-DoF_arm_gains.xml');
trajGen = HebiTrajectoryGenerator();

% Assume gravity points down in the frame of the first module.  This will
% get used for gravity compensation when controlling the arm.
gravityVec = [0 0 -1];

familyName = '6-DoF Arm';
moduleNames = {'Base','Shoulder','Elbow','Wrist1','Wrist2','Wrist3'};

group = HebiLookup.newGroupFromNames( familyName, moduleNames );
group.send('gains',gains);
cmd = CommandStruct();

group.startLog('dir','logs');

% Four Corners of a Box
xyzTargets = [ 0.20  0.50  0.50  0.20;    % x (m)
               0.20  0.20 -0.20 -0.20;    % y (m)
               0.20  0.20  0.20  0.20 ];  % z (m)

% Rotation matrix that makes the end-effector point straight forward
rotMatTarget = R_y(pi/2);
              
initPosition = [ 0 pi/4 pi/2 pi/4 -pi pi/2 ];

% Do IK to get joint position waypoints for each XYZ target, as well as the 
% desired orientation of the end effector.  Copy the first waypoint at the 
% end, so that it closes the loop
for i=1:length(xyzTargets)
    posTargets(i,:) = kin.getIK( 'xyz', xyzTargets(:,i), ...
                                 'SO3', rotMatTarget, ...
                                 'initial', initPosition ); 
end
posTargets(end+1,:) = posTargets(1,:);

% Get the initial feedback joint positions, and go from there to the first
% waypoint, using the trajectory API.  
fbk = group.getNextFeedback();

waypoints = [ fbk.position;
              posTargets(1,:) ];
timeToMove = 5;
time = [ 0 timeToMove ];

trajectory = trajGen.newJointMove( waypoints, 'time', time );
t0 = fbk.time;
t = 0;
while t < trajectory.getDuration
    
    fbk = group.getNextFeedback();
    t = fbk.time - t0;
    
    [pos,vel,acc] = trajectory.getState(t);
    
    gravCompEfforts = kin.getGravCompEfforts( fbk.position, gravityVec );
    dynamicCompEfforts = kin.getDynamicCompEfforts( fbk.position, ...
                                                    pos, vel, acc );
    
    cmd.position = pos;
    cmd.velocity = vel;
    cmd.effort = gravCompEfforts + dynamicCompEfforts;
    
    group.send(cmd);
end


% Go to all the different points.  
timeToMove = 3;
time = [ 0 timeToMove ];
    
for i=1:length(xyzTargets)
    
    waypoints = [ posTargets(i,:) ;
                  posTargets(i+1,:) ];

    trajectory = trajGen.newJointMove( waypoints, 'time', time );
    t0 = fbk.time;
    t = 0;
    while t < trajectory.getDuration

        fbk = group.getNextFeedback();
        t = fbk.time - t0;

        [pos,vel,acc] = trajectory.getState(t);

        gravCompEfforts = kin.getGravCompEfforts( fbk.position, gravityVec );
        dynamicCompEfforts = kin.getDynamicCompEfforts( fbk.position, ...
                                                        pos, vel, acc );

        cmd.position = pos;
        cmd.velocity = vel;
        cmd.effort = gravCompEfforts + dynamicCompEfforts;

        group.send(cmd);
    end
    
end

log = group.stopLog();

HebiUtils.plotLogs( log, 'position', 'figNum', 101 );
HebiUtils.plotLogs( log, 'velocity', 'figNum', 102 );
HebiUtils.plotLogs( log, 'effort', 'figNum', 103 );
