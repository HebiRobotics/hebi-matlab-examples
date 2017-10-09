 function [ gains ] = setGains_X5_1( gainGroup )
    
    feedbackFreq = gainGroup.getFeedbackFrequency();
    ledPauseTime = .1;

    gainGroup.setFeedbackFrequency(10);
    pause(0.2);
    
    numModules = gainGroup.getNumModules;

    params = getXSeriesControlParams( gainGroup );
    
    gains = GainStruct(); 
    
    %%%%%%%%%%%%%%%%%% 
    % Set STRATEGY_3 %
    %%%%%%%%%%%%%%%%%%
    gains.controlStrategy = ones(1, numModules) * 3;

    % Position ( rad ---> PWM )
    gains.positionKp = ones(1, numModules) * 1;
    gains.positionKi = ones(1, numModules) * .00;
    gains.positionIClamp = ones(1, numModules) * .2;
    gains.positionKd = ones(1, numModules) * 0;
    gains.positionFF = ones(1, numModules) * 0;
    gains.positionPunch = ones(1, numModules) * 0;
    gains.positionDeadZone = ones(1, numModules) * 0.00;
    gains.positionMinOutput = ones(1, numModules) * -1.0;
    gains.positionMaxOutput = ones(1, numModules) * 1.0;
    gains.positionMinTarget = ones(1, numModules) * -16*pi;
    gains.positionMaxTarget = ones(1, numModules) * 16*pi;
    gains.positionTargetLowpassGain = ones(1, numModules) * 1;
    gains.positionOutputLowpassGain = ones(1, numModules) * 1;
    gains.positionDOnError = ones(1, numModules) * 1;

    % Velocity ( rad/sec ---> PWM )
    gains.velocityKp = ones(1, numModules) * .05;
    gains.velocityKi = ones(1, numModules) * .000;
    gains.velocityIClamp = ones(1, numModules) * .0;
    gains.velocityKd = ones(1, numModules) * 0.0;
    gains.velocityFF = params.ffVelocities; 
    gains.velocityDeadZone = ones(1, numModules) * 0.00;
    gains.velocityPunch = ones(1, numModules) * 0.0;
    gains.velocityMinOutput = ones(1, numModules) * -1.0;
    gains.velocityMaxOutput = ones(1, numModules) * 1.0;
    gains.velocityMinTarget = -params.maxVelocities;
    gains.velocityMaxTarget = params.maxVelocities;
    gains.velocityTargetLowpassGain = ones(1, numModules) * 1;
    gains.velocityOutputLowpassGain = ones(1, numModules) * 0.75;
    gains.velocityDOnError = ones(1, numModules) * 1;

    % Torque ( N-m ---> PWM )
    gains.torqueKp = ones(1, numModules) * 0.5;
    gains.torqueKi = ones(1, numModules) * .000;
    gains.torqueIClamp = ones(1, numModules) * 0;
    gains.torqueKd = ones(1, numModules) * 1.0;
    gains.torqueFF = params.ffTorques;
    gains.torqueDeadZone = ones(1, numModules) * 0.00;
    gains.torquePunch = ones(1,numModules) * -1 .* ...
                            gains.torqueKp .* gains.torqueDeadZone;
    gains.torqueMinTarget = -params.maxTorques;
    gains.torqueMaxTarget = params.maxTorques;
    gains.torqueMinOutput = ones(1, numModules) * -1.0;
    gains.torqueMaxOutput = ones(1, numModules) * 1.0;
    gains.torqueTargetLowpassGain = ones(1, numModules) * 1;
    gains.torqueOutputLowpassGain = ones(1, numModules) * .5;
    gains.torqueDOnError = ones(1, numModules) * 0;


    % Set the gains
    gainGroup.set('gains', gains, 'led', 'y', 'persist', true); % PERSIST
    fprintf('Setting Strategy 3 Gains...');
    pause(ledPauseTime);
    gainGroup.set('led', []);
    pause(ledPauseTime);
    fprintf('DONE!\n');
    
    
    %%%%%%%%%%%%%%%%%% 
    % Set STRATEGY_4 %
    %%%%%%%%%%%%%%%%%%
    % Only the position gains change, since position is now going to the
    % torque loop.
    gains.controlStrategy = ones(1, numModules) * 4;

    % Position ( rad ---> N-m )
    gains.positionKp = ones(1, numModules) * 5;
    gains.positionKi = ones(1, numModules) * .000;
    gains.positionIClamp = ones(1, numModules) * .0;
    gains.positionKd = ones(1, numModules) * 0;
    gains.positionFF = ones(1, numModules) * 0;
    gains.positionPunch = ones(1, numModules) * 0;
    gains.positionDeadZone = ones(1, numModules) * 0.00;
    gains.positionMinOutput = -params.maxTorques;
    gains.positionMaxOutput = params.maxTorques;
    gains.positionMinTarget = ones(1, numModules) * -16*pi;
    gains.positionMaxTarget = ones(1, numModules) * 16*pi;
    gains.positionTargetLowpassGain = ones(1, numModules) * 1;
    gains.positionOutputLowpassGain = ones(1, numModules) * 1;
    gains.positionDOnError = ones(1, numModules) * 1;


    % Set the gains
    gainGroup.set('gains', gains, 'led', 'y', 'persist', true); % PERSIST
    fprintf('Setting Strategy 4 Gains...');
    pause(ledPauseTime);
    gainGroup.set('led', []);
    pause(ledPauseTime);
    fprintf('DONE!\n');

    
    %%%%%%%%%%%%%%%%%% 
    % Set STRATEGY_2 %
    %%%%%%%%%%%%%%%%%%
    % Only the velocity gains change, since velocity goes to torque loop.
    gains.controlStrategy = ones(1, numModules) * 2;

    % Velocity ( rad/sec ---> N-m )
    gains.velocityKp = ones(1, numModules) * .1;
    gains.velocityKi = ones(1, numModules) * .00;
    gains.velocityIClamp = ones(1, numModules) * 0.0;
    gains.velocityKd = ones(1, numModules) * 0.0;
    gains.velocityFF = ones(1, numModules) * 0.0; 
    gains.velocityDeadZone = ones(1, numModules) * 0.00;
    gains.velocityPunch = ones(1, numModules) * 0.0;
    gains.velocityMinOutput = -params.maxTorques;
    gains.velocityMaxOutput = params.maxTorques;
    gains.velocityMinTarget = -params.maxVelocities;
    gains.velocityMaxTarget = params.maxVelocities;
    gains.velocityTargetLowpassGain = ones(1, numModules) * 1;
    gains.velocityOutputLowpassGain = ones(1, numModules) * .50;
    gains.velocityDOnError = ones(1, numModules) * 1;



    % Set the gains
    gainGroup.set('gains', gains, 'led', 'y', 'persist', true); % PERSIST
    fprintf('Setting Strategy 2 Gains...');
    pause(ledPauseTime);
    gainGroup.set('led', []);
    pause(ledPauseTime);
    fprintf('DONE!\n');
    
    % Set the default control strategy
    gains = GainStruct();
    gains.controlStrategy = ones(1, numModules) * 3;
    gainGroup.set('gains', gains, 'led', 'y', 'persist', true); % PERSIST
    fprintf('Setting Default Control Strategy...');
    pause(ledPauseTime);
    gainGroup.set('led', []);
    pause(ledPauseTime);
    fprintf('DONE!\n');

    % Reset the feedback frequency to normal
    gainGroup.setFeedbackFrequency( feedbackFreq );
end
