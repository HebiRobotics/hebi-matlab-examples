 function [ gains ] = setSEASnakeGains_DirectPWM( gainGroup )

    import us.hebi.sdk.matlab.*;
    gains = GainStruct(); 
    
    % Lower the feedback frequency for more reliable communication.  We'll
    % set it back to what it was before we're done
    feedbackFreq = gainGroup.getFeedbackFrequency();
    gainGroup.setFeedbackFrequency(10);
    fbk = gainGroup.getNextFeedback;
    
    pause(0.2);

    numModules = gainGroup.getInfo.numModules;
    
    % Set Control Strategy
    gains.controlStrategy = ones(1, numModules) * 1; % DIRECT PWM
  
    % Set the spring constants
    %gainGroup.set('springconstant', ones(1, numModules) * 8.0);
    %gainGroup.set('springconstant', ones(1, numModules) * 12.0);
    pause(1.0);
    
    % Set the gains
    %gainGroup.set('gains', gains, 'persist', true, 'led', 'y'); % PERSIST
    gainGroup.set('gains', gains, 'led', 'y'); % NO PERSIST
    pause(1.0);

    gainGroup.set('led', []);
    gainGroup.setFeedbackFrequency(feedbackFreq);

end
