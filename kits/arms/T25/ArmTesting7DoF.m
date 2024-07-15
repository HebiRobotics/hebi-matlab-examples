% Reset the workspace
clear *;
close all;

HebiLookup.initialize();
pause(1.0);

lookupAddr = HebiLookup.getLookupAddresses();
lookupAddr{end+1} = '127.0.0.1';
HebiLookup.setLookupAddresses( lookupAddr );

enableLogging = true;

%%
%%%%%%%%%%%%%
% Arm Setup %
%%%%%%%%%%%%%

localDir = fileparts( mfilename('fullpath') );

% Load config file

configFile = fullfile( localDir, '6-DoF_arm.yaml' );
config = HebiUtils.loadRobotConfig( configFile );

group = HebiLookup.newGroupFromNames( config.families, config.names );

% Kinematic Model
kin = HebiUtils.loadHRDF(config.hrdf);

arm = HebiArm(group, kin);
arm.plugins = HebiArmPlugin.createFromConfigMap(config.plugins);

arm.update();

gains = HebiUtils.loadGains( config.gains.default );
HebiUtils.sendWithRetry(arm.group, 'gains', gains);

initialGoal = [0 0 0 pi/2 0 0];

mass = 3; % [kg]
com = [0 0 0]; % 1 [m] in x
kin.setPayload(mass, 'CoM', com);

arm.setGoal(initialGoal, 'time', 10);

while true
    arm.update();
    arm.send();
    
    disp(arm.state.fbk.effort);
end



%%
%%%%%%%%%%%
% Logging %
%%%%%%%%%%%

if enableLogging
   
   hebilogs{1} = arms{1}.group.stopLogFull();
   hebilogs{2} = arms{2}.group.stopLogFull();
   
   hebilog = hebilogs{1};
   
   % Plot tracking / error from the joints in the arm.  Note that there
   % will not by any 'error' in tracking for position and velocity, since
   % this example only commands effort.
   HebiUtils.plotLogs(hebilog, 'position');
   HebiUtils.plotLogs(hebilog, 'velocity');
   HebiUtils.plotLogs(hebilog, 'effort');
   
   % Plot the end-effectory trajectory and error
   kinematics_analysis( hebilog, arm.kin );
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   % Feel free to put more plotting code here %
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

