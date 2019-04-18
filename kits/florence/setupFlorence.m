function [ groups, kin, params ] = setupFlorence( robotName, ...
                                        controllerName, simulateFlag )

    % Re-initialize the lookup process
    HebiLookup.initialize();
    pause(1.0);

    % Get relative path
    localDir = fileparts(mfilename('fullpath'));
    params.localDir = localDir;

    % Load Robot Kinematics from HRDF
    kin.leg{1} = HebiKinematics([localDir '/hrdf/florence-leg-left.hrdf']);
    kin.leg{2} = HebiKinematics([localDir '/hrdf/florence-leg-right.hrdf']); 
    kin.chassis = HebiKinematics([localDir '/hrdf/florence-chassis.hrdf']); 

    params.jointNames = {'Hip1','Hip2','Hip3','Knee','Ankle1','Ankle2'};
    
    % Which body corresponds to the input frame to the first module.  This
    % is used to properly orient the hip IMUs to the chassis of the robot.
    params.hip1FrameNum = 3;

    params.gravityVec = [0; 0; -1];
    params.chassisMass = kin.chassis.getBodyMasses; % [kg]
    params.chassisFrame = kin.chassis.getFK('com',[]);
    params.chassisCoM = params.chassisFrame(1:3,4);  % [m]
    
    % Gas Spring for leg support
    params.springParams.baseForce = 20 * 4.45; % [N]
    params.springParams.freeLength = 0.39; % [m]
    params.springParams.compressedLength = 0.26; % [m]
    params.springParams.gasSpringDamp = [];
    
    % Bungee Cords for hip support
    params.bungeeParams.baseForce = 15 * 4.45; % [N]
    params.bungeeParams.momentArm = .0675; % [N] % 30mm standoff on bracket
    
    % Trajectory generator
    params.chassisTrajGen = HebiTrajectoryGenerator();
    params.rampTime = 0.5;

%     abductAngle = deg2rad( 7 );
%     kneeAngle = deg2rad( -60 );
%     hipAngle = kneeAngle / 2;
%     ankleAngle = hipAngle;
%     params.homeAngles(1,:) = ...
%         [ 0.01 -pi/2+abductAngle pi-hipAngle kneeAngle ankleAngle -abductAngle ];
    params.homeAngles(1,:) = [ -0.10, -1.70, 2.75, -1.43, -1.04, 0.13 ];
    params.homeAngles(2,:) = -params.homeAngles(1,:);
    
    % DEFAULT STANCE PARAMETERS
    params.stanceHeight = 0.75;    % [m]
    params.stanceWidth = 0.20;      % [m]
    params.stanceSkew = 0.05;       % [m]
    params.stanceMidX = -0.00;    % [m]
    
    params.footToeIn = 10;         % [deg]   
    params.footRot(:,:,1) = R_z( -deg2rad(params.footToeIn) );  % [SO3]
    params.footRot(:,:,2) = R_z( deg2rad(params.footToeIn) );  % [SO3]
    
    % DEFAULT STEP PARAMETERS (KINEMATIC TESTING)
    params.stepPeriod = 1.0;      % [sec]
    params.strideLength = 0.25;    % [m]
    params.swingLiftHeight = 0.03; % [m]
    params.swingMidpointPhase = 0.45; % [0-1]
    
    % % "PRETTY IN SIMULATION" STEP PARAMETERS (KINEMATIC TESTING)
    % params.stepPeriod = 0.66;      % [sec]
    % params.strideLength = 0.20;    % [m]
    % params.swingLiftHeight = 0.02; % [m]
    
    
    %%
    %%%%%%%%%%%%%%%
    % GROUP STUFF %
    %%%%%%%%%%%%%%%
    
    % Group for the legs of robot.  Setup assumes that all these modules
    % are up and running on the network.  If making this group fails the
    % function crashes.
    actuatorNames = {'leftHip1','leftHip2','leftHip3A','leftHip3B', ...
                     'leftKneeA','leftKneeB','leftAnkle1','leftAnkle2',...
                     'rightHip1','rightHip2','rightHip3A','rightHip3B', ...
                     'rightKneeA','rightKneeB','rightAnkle1','rightAnkle2'};
    params.actuatorNames = actuatorNames;
    
    % Indices for the 'main' modules, before doubling up for the hip and
    % knee.  These are the modules that match the kinematic chain as
    % defined in the HRDF.
    legIndices = [ 1 2 3   5   7 8];
    params.legIndex{1} = legIndices;
    params.legIndex{2} = legIndices + 8;
    
    % Group for the feet.  Setup assumes that all these modules
    % are up and running on the network.  If making this group fails the
    % function crashes.
    footNames = {'leftFoot','rightFoot'};
    params.footNames = footNames;
  
    if nargin>0 && ~isempty(robotName) && ~simulateFlag
        fprintf('Connecting to robot...');
        groups.legs = HebiLookup.newGroupFromNames( robotName, actuatorNames );
        groups.legs.setFeedbackFrequency( 200 );
        try
            groups.feet = HebiLookup.newGroupFromNames( robotName, footNames );
            groups.feet.setFeedbackFrequency( 200 );
        catch
            fprintf('NO FEET...');
            groups.feet = [];
        end
        fprintf('DONE.\n');
    else
        groups.legs = [];
        groups.feet = [];
    end
    
    % Group for the iPad / Mobile Device that's used for the controller.
    % This gets done continuously in a loops since it is likely that the
    % app is closed or the device is off the network when starting the
    % script.
    if nargin>1 && ~isempty(controllerName)
        fprintf('Connecting to controller...');
        retryDelayTime = 2.0;
        while true
            try
                groups.controller = ...
                    HebiLookup.newGroupFromNames( robotName, controllerName );
                fprintf('DONE.\n');
                break;
            catch
                disp(['  Did not find controller: ' robotName '|' controllerName]);
                pause( retryDelayTime );
            end
        end

        % Configure the iPad screen
        cmdIO = IoCommandStruct();
        cmdIO.a3 = 0;  % Set slider 'a3' to snap.
        cmdIO.e1 = 1;   % Make button 'b1' highlight, so we know it quits.
        cmdIO.b6 = 1;   % Make button 'b6' a toggle.
        
        if simulateFlag
            ledColor = 'b';
        else
            ledColor = 'g';
        end
            
        % Set the button configs, using acknowledgements.
        numSends = 0;
        maxSends = 10;
        ack = false;
        fprintf('Setting up to controller...');
        while ~ack
            ack = groups.controller.send( cmdIO, ...
                                          'led', ledColor, ...
                                          'ack', true );
            numSends = numSends + 1;
            if numSends > maxSends
                disp('Did not receive acknowledgement from controller.');
                break;
            end
        end
        fprintf('DONE.\n');
    else
        groups.controller = [];
    end
    
    
    %%
    %%%%%%%%%%%%%%%%%
    % SETTING GAINS %
    %%%%%%%%%%%%%%%%%
    params.xyzChassisGainsKp = [ 60; 20; 10; 10; 40; 10 ];
    params.xyzChassisGainsKd = [ 20; 10; 10; 2; 5; 2 ];                                                             
    
    params.xyzFootGainsKp = [ 100; 100; 100; 1; 1; 1 ];
    params.xyzFootGainsKd = [ 10; 10; 10; 1.0; 1.0; 1.0 ]; 
    
    if ~isempty(groups.legs)  
        % Load the gains, duplicating them for each item so that we only
        % have to specify one leg's worth of gains in the gain file.  Also 
        % remove the control strategy from the command gains so they can be 
        % sent without clearing the commands on the actuators.
        params.legGains = HebiUtils.loadGains( ...
                      [localDir '/gains/florence-leg-gains.xml'] );
        params.legSafetyParams = HebiUtils.loadSafetyParams( ...
                      [localDir '/gains/florence-leg-safety-params.xml'] );
        params.cmdGains = GainStruct();                              
        gainFields = fields(params.legGains);
        for i=1:length(gainFields)                    
            params.legGains.(gainFields{i}) = [ params.legGains.(gainFields{i}) params.legGains.(gainFields{i}) ];
            params.cmdGains.(gainFields{i}) = params.legGains.(gainFields{i});
        end
        params.cmdGains.controlStrategy = [];
        
        % Set the gains, using acknowledgements.
        numSends = 0;
        maxSends = 20;
        ack = false;
        while ~ack
            ack = groups.legs.send( 'gains', params.legGains, ...
                                    'safetyparams', params.legSafetyParams, ...
                                    'ack', true );
            numSends = numSends + 1;
            if numSends > maxSends
                error('Could not receive acknowledgement from at least 1 module');
            end
        end
    else
        params.legGains = [];
    end
 
end