 function [ fullGains ] = setHexapodGains( gainGroup, jointInds )
    
    feedbackFreq = gainGroup.getFeedbackFrequency();
    ledPauseTime = .3;

    gainGroup.setFeedbackFrequency(10);
    pause(0.2);
    
    localDir = fileparts(mfilename('fullpath'));
    
    legGains = HebiUtils.loadGains( ...
                    [localDir '/hexapod_3dof_leg_gains.xml' ] );
                
    numLegs = size(jointInds,1);
    
    fullGains = GainStruct();
    gainFields = fields(fullGains);
    
    % Expand out the gains to match the number of legs
    % Start at '2' to skip the time field, which doesn't expanded.
    for i=2:length(gainFields)
        fullGains.(gainFields{i}) = repmat( ...
                legGains.(gainFields{i}), 1, numLegs );
    end

    % Set the gains
    gainGroup.set('gains', gains, 'led', 'y');
    fprintf('Setting Leg Gains...');
    pause(ledPauseTime);
    gainGroup.set('led', []);
    pause(ledPauseTime);
    fprintf('DONE!\n');
    
    % Reset the feedback frequency to normal
    gainGroup.setFeedbackFrequency( feedbackFreq );
end
