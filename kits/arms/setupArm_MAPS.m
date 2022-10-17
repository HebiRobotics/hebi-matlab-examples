function [ arm, params ] = setupArm_MAPS()

    % NOTE: CHANGE AS NEEDED TO MATCH YOUR CONFIGURATION
    localDir = fileparts(mfilename('fullpath'));

    % Group Configuration
    familyName = 'MAPS';
       
    hrdfFileName = 'mapsArm_7DoF.hrdf';
    moduleNames = { 'J1-A'
                    'J2-B'
                    'J2-A'
                    'J3-B'
                    'J3-A'
                    'J4-B'
                    'J4-A' };

    armFbkFreq = 100;  % Hz
    try
        group = HebiLookup.newGroupFromNames(familyName, moduleNames);
        group.setFeedbackFrequency( armFbkFreq );
    catch
        disp('WARNING: Could not form arm group.');
        group = [];
    end

    % Kinematics
    kin = HebiKinematics( [localDir '/hrdf/' hrdfFileName] );

    % Make the Arm Object
    arm = HebiArm(group, kin);
    params = [];
    
end