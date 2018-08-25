function [ group, kin, gravityVec ] = setupArm_elbowScanner()

    % NOTE: CHANGE AS NEEDED TO MATCH YOUR CONFIGURATION

    % Group Configuration
    familyName = 'elbowScanningArm';
    moduleNames = { 'base'
                    'shoulder1'
                    'shoulder2'
                    'elbow1'
                    'elbow2'
                    'wrist1'
                    'wrist2'
                    'wrist3' };

    try
        group = HebiLookup.newGroupFromNames(familyName, moduleNames);
    catch
        disp('Building a group failed!  Returning an empty group.');
        group = [];
    end

    % Kinematics
    kin = HebiKinematics('elbowScanningArm.hrdf');

    % Determine gravity vector (assumes fixed base)
    if isempty( group )
        gravityVec = -[0 0 1];
    else
        fbk = group.getNextFeedback();
        gravityVec = -[fbk.accelX(1) fbk.accelY(1) fbk.accelZ(1)];

        gains = HebiUtils.loadGains('elbowScanningArmGains');
        group.send('gains',gains);
    end

end

