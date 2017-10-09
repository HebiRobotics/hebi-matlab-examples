 function [ ] = setSEASnakeGains_Default( moduleNames )
 
    gainGroup = HebiLookup.newGroupFromNames( '*', moduleNames );

    % Lower the feedback frequency for more reliable communication.  We'll
    % set it back to what it was before we're done
    feedbackFreq = gainGroup.getFeedbackFrequency();
    gainGroup.setFeedbackFrequency(10);
    
    pause(0.2);

    numModules = gainGroup.getInfo.numModules;
    
    % sysVoltage = mean(fbk.voltage);
    sysVoltage = 48;  % SEA Snake assumes 48V   
    
    % SEA Snake Geartrain Parameters
    gearEff = .75;
    gearRatio = 349;
    
    % SEA Snake Motor Parameters
    motorWindingRes = 69.9; % ohms        (36V Winding)
    motorTorqueConst = .0374; % Nm / A    (36V Winding)
    motorSpeedConst = 225; % RPM / V      (36V Winding)
%     motorWindingRes = 28.6; % ohms        (24V Winding)
%     motorTorqueConst = .0238; % Nm / A    (24V Winding)
%     motorSpeedConst = 402; % RPM / V      (24V Winding)

    % Max output motor velocity,
    maxVel = sysVoltage * motorSpeedConst * (2*pi/60) / gearRatio;  

    % Feed-forward gain for velocity (strategy 3/4)
    ffVel = gearRatio / ...
                  (motorSpeedConst * (2*pi/60) * sysVoltage);
    
    % Feed-forward gain for torque (all strategies)
    ffTor = motorWindingRes / (gearRatio*sysVoltage*motorTorqueConst*gearEff);
    
    
    %%%%%%%%%%%%%%%%%% 
    % Set STRATEGY_4 %
    %%%%%%%%%%%%%%%%%%
    gains = GainStruct(); 
    gains.controlStrategy = ones(1, numModules) * 4;

    % Position ( rad ---> N-m )
    gains.positionKp = ones(1, numModules) * 10;
    gains.positionKi = ones(1, numModules) * .000;
    gains.positionIClamp = ones(1, numModules) * 0;
    gains.positionKd = ones(1, numModules) * 0;
    gains.positionFF = ones(1, numModules) * 0;
    gains.positionPunch = ones(1, numModules) * 0;
    gains.positionDeadZone = ones(1, numModules) * 0.00;
    gains.positionMinOutput = ones(1, numModules) * -12;
    gains.positionMaxOutput = ones(1, numModules) * 12;
    gains.positionMinTarget = ones(1, numModules) * -(pi/2);
    gains.positionMaxTarget = ones(1, numModules) * (pi/2);
    gains.positionTargetLowpassGain = ones(1, numModules) * 1;
    gains.positionOutputLowpassGain = ones(1, numModules) * 1;
    gains.positionDOnError = ones(1, numModules) * 1;

    % Velocity ( rad/sec ---> PWM )
    gains.velocityKp = ones(1, numModules) * .1;
    gains.velocityKi = ones(1, numModules) * .0001;
    gains.velocityIClamp = ones(1, numModules) * .2;
    gains.velocityKd = ones(1, numModules) * 0.0;
    gains.velocityFF = ones(1, numModules) * ffVel; 
    gains.velocityDeadZone = ones(1, numModules) * 0.01;
    gains.velocityPunch = ones(1, numModules) * 0.0;
    gains.velocityMinOutput = ones(1, numModules) * -1.0;
    gains.velocityMaxOutput = ones(1, numModules) * 1.0;
    gains.velocityMinTarget = ones(1, numModules) * -1.25*maxVel;
    gains.velocityMaxTarget = ones(1, numModules) * 1.25*maxVel;
    gains.velocityTargetLowpassGain = ones(1, numModules) * 1;
    gains.velocityOutputLowpassGain = ones(1, numModules) * .5;
    gains.velocityDOnError = ones(1, numModules) * 1;

    % Torque ( N-m ---> PWM )
    gains.torqueKp = ones(1, numModules) * .75;
    gains.torqueKi = ones(1, numModules) * 0;
    gains.torqueIClamp = ones(1, numModules) * 0;
    gains.torqueKd = ones(1, numModules) * 1;
    gains.torqueFF = ones(1, numModules) * ffTor;
    gains.torqueDeadZone = ones(1, numModules) * 0.01;
    gains.torquePunch = ones(1,numModules) * 0;
    gains.torqueMinTarget = ones(1, numModules) * -12;
    gains.torqueMaxTarget = ones(1, numModules) * 12;
    gains.torqueMinOutput = ones(1, numModules) * -1.0;
    gains.torqueMaxOutput = ones(1, numModules) * 1.0;
    gains.torqueTargetLowpassGain = ones(1, numModules) * 1;
    gains.torqueOutputLowpassGain = ones(1, numModules) * .15;
    gains.torqueDOnError = ones(1, numModules) * 0;
    
    % Set the gains
    gainGroup.set('gains', gains, 'led', 'y', 'persist', true); % PERSIST
    fprintf('Setting Strategy 4 Gains...');
    pause(.25);
    gainGroup.set('led', []);
    pause(.25);
    fprintf('DONE!\n');
 
    
    
    %%%%%%%%%%%%%%%%%% 
    % Set STRATEGY_3 %
    %%%%%%%%%%%%%%%%%%
    gains = GainStruct(); 
    gains.controlStrategy = ones(1, numModules) * 3;

    % Position ( rad ---> N-m )
    gains.positionKp = ones(1, numModules) * 5;
    gains.positionKi = ones(1, numModules) * .000;
    gains.positionIClamp = ones(1, numModules) * 0;
    gains.positionKd = ones(1, numModules) * 0;
    gains.positionFF = ones(1, numModules) * 0;
    gains.positionPunch = ones(1, numModules) * 0;
    gains.positionDeadZone = ones(1, numModules) * 0.00;
    gains.positionMinOutput = ones(1, numModules) * -12;
    gains.positionMaxOutput = ones(1, numModules) * 12;
    gains.positionMinTarget = ones(1, numModules) * -(pi/2);
    gains.positionMaxTarget = ones(1, numModules) * (pi/2);
    gains.positionTargetLowpassGain = ones(1, numModules) * 1;
    gains.positionOutputLowpassGain = ones(1, numModules) * 1;
    gains.positionDOnError = ones(1, numModules) * 1;

    % Velocity ( rad/sec ---> PWM )
    gains.velocityKp = ones(1, numModules) * .1;
    gains.velocityKi = ones(1, numModules) * .0001;
    gains.velocityIClamp = ones(1, numModules) * .2;
    gains.velocityKd = ones(1, numModules) * 0.0;
    gains.velocityFF = ones(1, numModules) * ffVel; 
    gains.velocityDeadZone = ones(1, numModules) * 0.01;
    gains.velocityPunch = ones(1, numModules) * 0.0;
    gains.velocityMinOutput = ones(1, numModules) * -1.0;
    gains.velocityMaxOutput = ones(1, numModules) * 1.0;
    gains.velocityMinTarget = ones(1, numModules) * -1.25*maxVel;
    gains.velocityMaxTarget = ones(1, numModules) * 1.25*maxVel;
    gains.velocityTargetLowpassGain = ones(1, numModules) * 1;
    gains.velocityOutputLowpassGain = ones(1, numModules) * .5;
    gains.velocityDOnError = ones(1, numModules) * 1;

    % Torque ( N-m ---> PWM )
    gains.torqueKp = ones(1, numModules) * .75;
    gains.torqueKi = ones(1, numModules) * 0;
    gains.torqueIClamp = ones(1, numModules) * 0;
    gains.torqueKd = ones(1, numModules) * 1;
    gains.torqueFF = ones(1, numModules) * ffTor;
    gains.torqueDeadZone = ones(1, numModules) * 0.01;
    gains.torquePunch = ones(1,numModules) * 0;
    gains.torqueMinTarget = ones(1, numModules) * -12;
    gains.torqueMaxTarget = ones(1, numModules) * 12;
    gains.torqueMinOutput = ones(1, numModules) * -1.0;
    gains.torqueMaxOutput = ones(1, numModules) * 1.0;
    gains.torqueTargetLowpassGain = ones(1, numModules) * 1;
    gains.torqueOutputLowpassGain = ones(1, numModules) * .15;
    gains.torqueDOnError = ones(1, numModules) * 0;
    
    % Set the gains
    gainGroup.set('gains', gains, 'led', 'y'); % NO PERSIST
    fprintf('Setting Strategy 3 Gains...');
    pause(.25);
    gainGroup.set('led', []);
    pause(.25);
    fprintf('DONE!\n');
    
    
    
    %%%%%%%%%%%%%%%%%% 
    % Set STRATEGY_2 %
    %%%%%%%%%%%%%%%%%%
    gains = GainStruct(); 
    gains.controlStrategy = ones(1, numModules) * 2;

    % Position ( rad ---> N-m )
    gains.positionKp = ones(1, numModules) * 10;
    gains.positionKi = ones(1, numModules) * .000;
    gains.positionIClamp = ones(1, numModules) * 0;
    gains.positionKd = ones(1, numModules) * 0;
    gains.positionFF = ones(1, numModules) * 0;
    gains.positionPunch = ones(1, numModules) * 0;
    gains.positionDeadZone = ones(1, numModules) * 0.00;
    gains.positionMinOutput = ones(1, numModules) * -12;
    gains.positionMaxOutput = ones(1, numModules) * 12;
    gains.positionMinTarget = ones(1, numModules) * -(pi/2);
    gains.positionMaxTarget = ones(1, numModules) * (pi/2);
    gains.positionTargetLowpassGain = ones(1, numModules) * 1;
    gains.positionOutputLowpassGain = ones(1, numModules) * 1;
    gains.positionDOnError = ones(1, numModules) * 1;

    % Velocity ( rad/sec ---> PWM )
    gains.velocityKp = ones(1, numModules) * .3;
    gains.velocityKi = ones(1, numModules) * .0001;
    gains.velocityIClamp = ones(1, numModules) * 1;
    gains.velocityKd = ones(1, numModules) * 0.0;
    gains.velocityFF = ones(1, numModules) * 0.0; 
    gains.velocityDeadZone = ones(1, numModules) * 0.01;
    gains.velocityPunch = ones(1, numModules) * 0.0;
    gains.velocityMinOutput = ones(1, numModules) * -1.0;
    gains.velocityMaxOutput = ones(1, numModules) * 1.0;
    gains.velocityMinTarget = ones(1, numModules) * -1.25*maxVel;
    gains.velocityMaxTarget = ones(1, numModules) * 1.25*maxVel;
    gains.velocityTargetLowpassGain = ones(1, numModules) * 1;
    gains.velocityOutputLowpassGain = ones(1, numModules) * 1;
    gains.velocityDOnError = ones(1, numModules) * 1;

    % Torque ( N-m ---> PWM )
    gains.torqueKp = ones(1, numModules) * .75;
    gains.torqueKi = ones(1, numModules) * 0;
    gains.torqueIClamp = ones(1, numModules) * 0;
    gains.torqueKd = ones(1, numModules) * 1;
    gains.torqueFF = ones(1, numModules) * ffTor;
    gains.torqueDeadZone = ones(1, numModules) * 0.01;
    gains.torquePunch = ones(1,numModules) * 0;
    gains.torqueMinTarget = ones(1, numModules) * -12;
    gains.torqueMaxTarget = ones(1, numModules) * 12;
    gains.torqueMinOutput = ones(1, numModules) * -1.0;
    gains.torqueMaxOutput = ones(1, numModules) * 1.0;
    gains.torqueTargetLowpassGain = ones(1, numModules) * 1;
    gains.torqueOutputLowpassGain = ones(1, numModules) * .15;
    gains.torqueDOnError = ones(1, numModules) * 0;
    
    
    % Set the gains
    gainGroup.set('gains', gains, 'led', 'y'); % NO PERSIST
    fprintf('Setting Strategy 2 Gains...');
    pause(.25);
    gainGroup.set('led', []);
    pause(.25);
    fprintf('DONE!\n');
    
    % Set the default control strategy to strategy 3?
    gains = GainStruct(); 
    gains.controlStrategy = ones(1, numModules) * 3;
    
    gainGroup.set('gains', gains, 'persist', true, 'led', 'y'); % PERSIST
    fprintf( 'Activating Strategy %d...', gains.controlStrategy(1) );
    pause(.25);
    gainGroup.set('led', []);
    pause(.25);
    fprintf('DONE!\n');
    
    % Restore the original feedback frequency
    gainGroup.setFeedbackFrequency(feedbackFreq);

end
