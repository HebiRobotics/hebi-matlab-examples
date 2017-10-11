function [ params ] = getXSeriesControlParams( group )
%Getting all the configuration dependent parameters of the X5 Modules
    %   Detailed explanation goes here ...

    feedbackFreq = group.getFeedbackFrequency();

    group.setFeedbackFrequency(10);
%     fbk = group.getNextFeedback;
    pause(0.1);
    
    groupInfo = group.getInfo();  
    numModules = group.getNumModules();
    
%     % Adjust feed-forward velocity gain based on voltage
%     sysVoltage = fbk.voltage;
    
    % Gains are specified at 48V and the modules automatically compensate
    % if the bus voltage is different.
    sysVoltage = 48 * ones(1,numModules);
    
    params.maxTorques = nan(1,numModules);
    params.maxVelocities = nan(1,numModules);
    params.ffTorques = nan(1,numModules);
    params.ffVelocities = nan(1,numModules);
    
    maxMotorSpeed = 25000 *(2*pi) / 60;  % RPM ---> rad/sec
    
    mechanicalType = groupInfo.mechanicalType;
    mechanicalRevision = groupInfo.mechanicalRevision; 

    for i=1:numModules
        switch mechanicalType{i}
            case 'X5-1'
                params.maxTorques(i) = 4;
                gearRatio = 272.222;
                gearEff = .65; % Guessed
            case 'X5-4'
                params.maxTorques(i) = 10;
                gearRatio = 762.222;
                gearEff = .65; % Guessed
            case 'X5-9'
                params.maxTorques(i) = 20;
                gearRatio = 1742.222;
                gearEff = .65; % Guessed    
            case 'X8-3'
                params.maxTorques(i) = 10;
                gearRatio = 272.222;
                gearEff = .65; % Guessed
            case 'X8-9'
                params.maxTorques(i) = 20;
                gearRatio = 762.222;
                gearEff = .65; % Guessed
            case 'X8-16'
%                 params.maxTorques(i) = 40;
%                 gearRatio = 1742.222;
%                 gearEff = .65; % Guessed
        end
        
        if strfind(mechanicalType{i},'X5')
            switch mechanicalRevision{i}
                case {'A9V','B9V','C9V'}
                    % Motor Parameters (Maxon EC-Max-5W 9V)
                    terminalResistance = 9.99; % ohms
                    windingInductance = 0.163e-3; % henrys
                    windingTorqueConstant = .00626; % Nm / A
                    windingSpeedConstant = 1530; % RPM / V
                case {'A12V','B12V','C12V'} 
                    % Motor Parameters (Maxon EC-Max-5W 12V)
                    terminalResistance = 15.7; % ohms
                    windingInductance = 0.254e-3; % henrys
                    windingTorqueConstant = .0078; % Nm / A
                    windingSpeedConstant = 1220; % RPM / V     
            end
        elseif strfind(mechanicalType{i},'X8')
            switch mechanicalRevision{i}
                case {'A12V'} 
                    % Motor Parameters (Maxon EC-Max-5W 12V)
                    terminalResistance = 5.3; % ohms
                    windingInductance = 0.14e-3; % henrys
                    windingTorqueConstant = .00932; % Nm / A
                    windingSpeedConstant = 1020; % RPM / V     
            end
        end
        
        params.maxVelocities(i) = maxMotorSpeed / gearRatio;
        
        params.ffTorques(i) = terminalResistance / ...
                    (gearRatio*sysVoltage(i)*windingTorqueConstant*gearEff);
                
        params.ffVelocities(i) = gearRatio / ...
                    (windingSpeedConstant*(2*pi/60)*sysVoltage(i));        
    end

    group.setFeedbackFrequency(feedbackFreq);
end
