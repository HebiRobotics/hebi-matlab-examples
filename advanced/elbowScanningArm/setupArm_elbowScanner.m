function [ group, kin, gravityVec ] = setupArm_elbowScanner()

    % NOTE: CHANGE AS NEEDED TO MATCH YOUR CONFIGURATION

    % Communications
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
    probeFrame = eye(4);
    probeFrame(1:3,1:3) = R_y(pi/2);
    probeFrame(1:3,4) = [ 0.120; 
                          0.0;
                          0.0175 ];

    kin = HebiKinematics();
    kin.addBody('X5-9');
    kin.addBody('X5-HeavyBracket', 'mount', 'left-outside');
    kin.addBody('X8-16');
    kin.addBody('X5-Link', 'extension', .275, 'twist', pi/2);
    kin.addBody('X5-9');
    kin.addBody('X5-Link', 'extension', .175, 'twist', pi/2);
    kin.addBody('X5-9');
    kin.addBody('X5-Link', 'extension', .225, 'twist', pi);
    kin.addBody('X5-9');
    kin.addBody('X5-Link', 'extension', .225, 'twist', pi);
    kin.addBody('X5-4');
    kin.addBody('X5-LightBracket', 'mount', 'right');
    kin.addBody('X5-4');
    kin.addBody('X5-LightBracket', 'mount', 'right');
    kin.addBody('X5-4');
    kin.addBody('GenericLink','output',probeFrame,'mass',.100,'com',[0;0;0]);



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

