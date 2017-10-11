 function [ gains ] = setHexapodGains( gainGroup, jointInds )
    
    feedbackFreq = gainGroup.getFeedbackFrequency();
    ledPauseTime = .3;

    gainGroup.setFeedbackFrequency(10);
    pause(0.2);
    
    numModules = gainGroup.getNumModules;

    params = getXSeriesControlParams( gainGroup );
    
    gains = GainStruct(); 
    
    %%%%%%%%%%%%%%%%%% 
    % Set STRATEGY_3 %
    %%%%%%%%%%%%%%%%%%
    for leg=1:size(jointInds,1)

        % Control Strategy
        gains.controlStrategy(jointInds(leg,:)) = ones(1, 3) * 3;

        % Position ( rad ---> PWM )
        gains.positionKp(jointInds(leg,:)) = [5 8 2];
        gains.positionKi(jointInds(leg,:)) = ones(1,3) * .00;
        gains.positionIClamp(jointInds(leg,:)) = ones(1,3) * 0;
        gains.positionKd(jointInds(leg,:)) = ones(1,3) * 0;
        gains.positionFF(jointInds(leg,:)) = ones(1,3) * 0;
        gains.positionPunch(jointInds(leg,:)) = ones(1,3) * 0;
        gains.positionDeadZone(jointInds(leg,:)) = ones(1,3) * 0.00;
        gains.positionMinOutput(jointInds(leg,:)) = ones(1,3) * -1.0;
        gains.positionMaxOutput(jointInds(leg,:)) = ones(1,3) * 1.0;
        gains.positionMinTarget(jointInds(leg,:)) = ones(1,3) * -16*pi;
        gains.positionMaxTarget(jointInds(leg,:)) = ones(1,3) * 16*pi;
        gains.positionTargetLowpassGain(jointInds(leg,:)) = ones(1,3) * 1;
        gains.positionOutputLowpassGain(jointInds(leg,:)) = ones(1,3) * 1;
        gains.positionDOnError(jointInds(leg,:)) = ones(1,3) * 1;

        % Velocity ( rad/sec ---> PWM )
        gains.velocityKp(jointInds(leg,:)) = [.1 .2 .1] / 2;
        gains.velocityKi(jointInds(leg,:)) = ones(1,3) * .000;
        gains.velocityIClamp(jointInds(leg,:)) = ones(1,3) * .0;
        gains.velocityKd(jointInds(leg,:)) = ones(1,3) * 0.0;
        gains.velocityFF(jointInds(leg,:)) = params.ffVelocities(jointInds(leg,:)); 
        gains.velocityDeadZone(jointInds(leg,:)) = ones(1,3) * 0.00;
        gains.velocityPunch(jointInds(leg,:)) = ones(1,3) * 0.0;
        gains.velocityMinOutput(jointInds(leg,:)) = ones(1,3) * -1.0;
        gains.velocityMaxOutput(jointInds(leg,:)) = ones(1,3) * 1.0;
        gains.velocityMinTarget(jointInds(leg,:)) = -params.maxVelocities(jointInds(leg,:));
        gains.velocityMaxTarget(jointInds(leg,:)) = params.maxVelocities(jointInds(leg,:));
        gains.velocityTargetLowpassGain(jointInds(leg,:)) = ones(1,3) * 1;
        gains.velocityOutputLowpassGain(jointInds(leg,:)) = ones(1,3) * 0.75;
        gains.velocityDOnError(jointInds(leg,:)) = ones(1,3) * 1;

        % Torque ( N-m ---> PWM )
        gains.torqueKp(jointInds(leg,:)) = ones(1,3) * 0.25;
        gains.torqueKi(jointInds(leg,:)) = ones(1,3) * .000;
        gains.torqueIClamp(jointInds(leg,:)) = ones(1,3) * 0;
        gains.torqueKd(jointInds(leg,:)) = ones(1,3) * 0.5;
        gains.torqueFF(jointInds(leg,:)) = params.ffTorques(jointInds(leg,:));
        gains.torqueDeadZone(jointInds(leg,:)) = ones(1,3) * 0.00;
        gains.torquePunch(jointInds(leg,:)) = -1 .* ...
          gains.torqueKp(jointInds(leg,:)) .* gains.torqueDeadZone(jointInds(leg,:));
        gains.torqueMinTarget(jointInds(leg,:)) = -params.maxTorques(jointInds(leg,:));
        gains.torqueMaxTarget(jointInds(leg,:)) = params.maxTorques(jointInds(leg,:));
        gains.torqueMinOutput(jointInds(leg,:)) = ones(1,3) * -1.0;
        gains.torqueMaxOutput(jointInds(leg,:)) = ones(1,3) * 1.0;
        gains.torqueTargetLowpassGain(jointInds(leg,:)) = ones(1,3) * 1;
        gains.torqueOutputLowpassGain(jointInds(leg,:)) = ones(1,3) * .5;
        gains.torqueDOnError(jointInds(leg,:)) = ones(1,3) * 0;
    end

    % Set the gains
    gainGroup.set('gains', gains, 'led', 'y'); % PERSIST
    fprintf('Setting Strategy 3 Gains...');
    pause(ledPauseTime);
    gainGroup.set('led', []);
    pause(ledPauseTime);
    fprintf('DONE!\n');
    
    
    %%%%%%%%%%%%%%%%%% 
    % Set STRATEGY_4 %
    %%%%%%%%%%%%%%%%%%
    for leg=1:size(jointInds,1)
        % Only the position gains change, since position is now going to the
        % torque loop.
        gains.controlStrategy(jointInds(leg,:)) = ones(1,3) * 4;

        % Position ( rad ---> N-m )
        gains.positionKp(jointInds(leg,:)) = [15 30 15];
        gains.positionKi(jointInds(leg,:)) = ones(1,3) * .000;
        gains.positionIClamp(jointInds(leg,:)) = ones(1,3) * .0;
        gains.positionKd(jointInds(leg,:)) = ones(1,3) * 0;
        gains.positionFF(jointInds(leg,:)) = ones(1,3) * 0;
        gains.positionPunch(jointInds(leg,:)) = ones(1,3) * 0;
        gains.positionDeadZone(jointInds(leg,:)) = ones(1,3) * 0.00;
        gains.positionMinOutput(jointInds(leg,:)) = -params.maxTorques(jointInds(leg,:));
        gains.positionMaxOutput(jointInds(leg,:)) = params.maxTorques(jointInds(leg,:));
        gains.positionMinTarget(jointInds(leg,:)) = ones(1,3) * -16*pi;
        gains.positionMaxTarget(jointInds(leg,:)) = ones(1,3) * 16*pi;
        gains.positionTargetLowpassGain(jointInds(leg,:)) = ones(1,3) * 1;
        gains.positionOutputLowpassGain(jointInds(leg,:)) = ones(1,3) * 1;
        gains.positionDOnError(jointInds(leg,:)) = ones(1,3) * 1;
    end

    % Set the gains
    gainGroup.set('gains', gains, 'led', 'y'); % PERSIST
    fprintf('Setting Strategy 4 Gains...');
    pause(ledPauseTime);
    gainGroup.set('led', []);
    pause(ledPauseTime);
    fprintf('DONE!\n');
    
    
    % Set the default control strategy
    gains = GainStruct();
    gains.controlStrategy = ones(1, numModules) * 4;
    gainGroup.set('gains', gains, 'led', 'y'); % PERSIST
    fprintf('Setting Default Control Strategy...');
    pause(ledPauseTime);
    gainGroup.set('led', []);
    pause(ledPauseTime);
    fprintf('DONE!\n');

    % Reset the feedback frequency to normal
    gainGroup.setFeedbackFrequency( feedbackFreq );
end
