% Generate a trajectory and execute it using the non-blocking API.  This
% code does the exact same commands as the previous blocking API example.
%
% For more information type:
%    help HebiTrajectoryGenerator
%    help HebiTrajectory
%
% This script assumes you can create a group with 1 module.
%
% HEBI Robotics
% June 2018

%% Setup Group
clear *;
close all;
HebiLookup.initialize();

familyName = 'Andy';
%moduleNames = 'J1';
%group = HebiLookup.newGroupFromNames( familyName, moduleNames );

HebiLookup.clearModuleList();
pause(1.0);


kin = HebiUtils.loadHRDF('hrdf/T8-16LoadedEnduraceTest.hrdf')
%kin = HebiUtils.loadHRDF('hrdf/T-16PlusLoadedEnduraceTest.hrdf')


% Keyboard input
kb = HebiKeyboard();
localDir = fileparts(mfilename('fullpath'));
keys = read(kb);
abortFlag = false;

% Individual groups for logging
logFeedbackRate = 20; % Hz 

% Group containing all modules
batchGroup = HebiLookup.newGroupFromFamily(familyName); 
numModules = batchGroup.getNumModules; %No. of modules in family
names = batchGroup.getInfo.name; % Names for making individual log groups
serialNumbers = batchGroup.getInfo.serialNumber; % For saving log files

for i=1:numModules 
    logGroup{i} = HebiLookup.newGroupFromNames(familyName, names(i));
    logGroup{i}.setFeedbackFrequency( logFeedbackRate );

end

localDir = fileparts(mfilename('fullpath'));
fileDir = [localDir '/logs/' familyName];

% Start logs for individual units.  Save the file path so that we can
% load it later and compare to the last log.
for i=1:1:numModules 
    startLogPath{i} = ...
        logGroup{i}.startLog('dir',[fileDir, names{i}] );
end 

%% Non-Blocking Trajectory
trajGen = HebiTrajectoryGenerator();
cmd = CommandStruct();
batchGroup.startLog( 'dir', 'logs' );

% setting the start position
fbk=batchGroup.getNextFeedback();
waypoints = [
    fbk.position;
    0 ];
time = [ 0 6];
trajectory = trajGen.newJointMove( waypoints, 'time', time );

t0 = tic();
t = toc(t0);


% %Time keeping
% timeStart = tic;
timeStart = tic();
timeSinceLastPrint = tic();
timeSinceLastLog = tic();

% Set reporting/logging parameters
printTime = 1200; %1200 [s] = 20 minutes
logDuration = 3600; %3600 [s] = 1 hour

while keys.ESC == 0
       
    lastKeys = keys;
    keys = read(kb);
    
    
        while t < trajectory.getDuration()

            fbk = batchGroup.getNextFeedback();

            % Get trajectory state at the current time

            [pos, vel, accel] = trajectory.getState(t);

            % Send the commands
            cmd.position = pos;
            cmd.velocity = vel;
            batchGroup.send(cmd);
            t = toc(t0);  
        end


        % Go from 0 to 5 revolutions in 30 seconds
        waypoints = [
            0;
            2*pi ];
        time = [ 0 10];

        % This function generates smooth minimum jerk trajectories
        trajectory = trajGen.newJointMove( waypoints, 'time', time );

        % Visualize the trajectory
        %HebiUtils.plotTrajectory(trajectory);
        %drawnow;

        t0 = tic();
        t = toc(t0);
        while t < trajectory.getDuration()

            fbk = batchGroup.getNextFeedback();

            % Get trajectory state at the current time

            [pos, vel, accel] = trajectory.getState(t);

            % Send the commands
            cmd.position = pos;
            cmd.velocity = vel;
            batchGroup.send(cmd);
            t = toc(t0);  
        end

        % Reverse the waypoints and to go back to the first position
        waypoints = flipud(waypoints);
        trajectory = trajGen.newJointMove( waypoints, 'time', time );

        t0 = tic();
        t = toc(t0);
        while t < trajectory.getDuration()

            fbk = batchGroup.getNextFeedback();

            % Get trajectory state at the current time

            [pos, vel, accel] = trajectory.getState(t);

            % Send the commands
            cmd.position = pos;
            cmd.velocity = vel;
            batchGroup.send(cmd);
            t = toc(t0); 
        end
        
        
         %Report status
        if toc(timeSinceLastPrint) > printTime
           disp(['Total run-time: ',num2str(toc(timeStart)/3600),' hours.']);
           timeSinceLastPrint = tic; %Reset print timer
        end
    
        %Start new logs every hour for "alive" modules
        if toc(timeSinceLastLog) > logDuration
            for i=1:numModules
                logGroup{i}.startLog('dir',[fileDir, names{i}] );
            end
           timeSinceLastLog=tic;
        end
    
           % Check for keyboard input and break out of the main loop
   % if the ESC key is pressed.
   % keys = read(kb);
   % abortFlag = keys.ESC;
end
% Stop logging and plot the velocity data using helper functions
log = batchGroup.stopLog();
HebiUtils.plotLogs( log, 'position', 'figNum', 101 );
HebiUtils.plotLogs( log, 'velocity', 'figNum', 102 );

