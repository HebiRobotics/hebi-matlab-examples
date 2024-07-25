% Reset the workspace
clear *;
close all;

% NOTE: CHANGE AS NEEDED TO MATCH YOUR CONFIGURATION
localDir = fileparts(mfilename('fullpath'));

HebiLookup.initialize();
pause(1.0);

lookupAddr = HebiLookup.getLookupAddresses();
lookupAddr{end+1} = '127.0.0.1';
HebiLookup.setLookupAddresses( lookupAddr );

enableLogging = true;

%%
%%%%%%%%%%%%%%%%%%%%%%%
% Mobile Device Setup %
%%%%%%%%%%%%%%%%%%%%%%%
controlFamily = 'T25_6-DoF_arm';
deviceName = 'MobileIO';
quitDemoButton = 'b8';
jogX_axis = 'a1';
jogY_axis = 'a2';
jogZ_axis = 'a8';

cmd = CommandStruct();

% Setup mobileIO UI

unicode = @(input) sprintf(strrep(input, '\u', '\x'));

mobileIO = HebiMobileIO.findDevice( controlFamily, deviceName );
mobileIO.initializeUI();
mobileIO.setButtonLabel( 8, 'Quit' );
mobileIO.setAxisLabel( [1 2 8], {'X','Y','Z'} );
mobileIO.setAxisSnap([3 4 5 6], [0 0 0 0]);

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

initialGoal = [0 0 0 0 0 0];

mass = 1; % [kg]
com = [0 0 0]; % 1 [m] in x
kin.setPayload(mass, 'CoM', com);

arm.setGoal(initialGoal, 'time', 10);

while ~arm.isAtGoal
    arm.update();
    arm.send();
end

xyzTarget = arm.kin.getFK('endeffector', arm.state.cmdPos);
xyzTarget = xyzTarget(1:3,4);

%% Startup
abortFlag = false;

printTimer = tic;

%%%%%%%%%%%%%%%%
% Main Routine %
%%%%%%%%%%%%%%%%
    
while ~abortFlag

    %%%%%%%%%%%%%%%%%%%
    % Gather Feedback %
    %%%%%%%%%%%%%%%%%%%
    try
        arm.update();
        arm.send();
    catch
        disp('Could not get robot feedback!');
        break;
    end

    % Grab initial pose from phone
    [hasNewFeedback, feedbackAge] = mobileIO.update('timeout',0);

    % Abort goal updates if the phone didn't respond
    if hasNewFeedback
        [fbkMobile, fbkIO] = mobileIO.getFeedback();
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Read/Map Joystick Inputs %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Check for quit command
    if fbkIO.(quitDemoButton)
        abortFlag = true;
        disp('Quitting the Demo.');
        break;
    end



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Arm Joystick Position Control %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    jogScale = 0.003;

    jogX = jogScale * fbkIO.(jogX_axis);
    jogY = jogScale * fbkIO.(jogY_axis);
    jogZ = jogScale * fbkIO.(jogZ_axis);

    moveXYZ = [jogX; jogY; jogZ];

    newXYZ = xyzTarget + moveXYZ;

    dist = norm(newXYZ);
    distXY = norm([newXYZ(1) newXYZ(2) 0]);
    
    xyzTarget = xyzTarget + moveXYZ;


    seedPosIK = arm.state.cmdPos;

     % Find target using inverse kinematics
    ikPosition = arm.kin.getIK('xyz', xyzTarget, ...
                                   'tipAxis', [0 0 -1], ...
                                   'initial', seedPosIK, ...
                                   'MaxIter', 20 );

    arm.setGoal(ikPosition, 'time', 0.5); 
    goalPosition = ikPosition;
    
    % print current position
    
    if toc(printTimer) > 1
        currentPos = arm.kin.getFK('endeffector', arm.state.fbk.position);
        disp(currentPos(1:3, 4));
        printTimer = tic;
    end

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

