function [ group, kin, gravityVec ] = setupArm()

% NOTE: CHANGE AS NEEDED TO MATCH YOUR CONFIGURATION

% Communications
group = HebiLookup.newGroupFromNames('Arm', {
   'base'
   'shoulder1'
   'shoulder2'
   'elbow1'
   'elbow2'
   'wrist'
});

CoM = [0 0 0];

A_2074_01_OutputTransform = [ 0  0 -1  0;
                              0  1  0  0; 
                              1  0  0  0;
                              0  0  0  1 ];

A_2131_08_OutputTransform = [ 0  0  1 .090;
                              0  1  0   0; 
                             -1  0  0 -.065;
                              0  0  0   1 ];

trocharOutputTransform = [ 1  0  0   0;
                           0  1  0  -.00; 
                           0  0  1  .100;
                           0  0  0   1 ];


% Kinematics
kin = HebiKinematics();
kin.addBody('X8-16');
kin.addBody('GenericLink', 'CoM', CoM, 'Output', A_2074_01_OutputTransform, 'mass', 0.120);
kin.addBody('X5-Link', 'extension', .175, 'twist', 0);
kin.addBody('X8-16');
kin.addBody('GenericLink', 'CoM', CoM,'Output', A_2131_08_OutputTransform, 'mass', 0.300);
kin.addBody('X8-16');
kin.addBody('GenericLink', 'CoM', CoM, 'Output', A_2074_01_OutputTransform, 'mass', 0.120);
kin.addBody('X5-Link', 'extension', .325, 'twist', 0);
kin.addBody('X8-9');
kin.addBody('GenericLink', 'CoM', CoM,'Output', A_2131_08_OutputTransform, 'mass', 0.300);
kin.addBody('X8-9');
kin.addBody('GenericLink', 'CoM', CoM, 'Output', A_2074_01_OutputTransform, 'mass', 0.120);
kin.addBody('X5-Link', 'extension', .325, 'twist', 0);
kin.addBody('X5-4');
kin.addBody('X5-LightBracket', 'mount', 'right');
kin.addBody('GenericLink', 'CoM', CoM, 'Output', trocharOutputTransform, 'mass', 0.010);

% Determine gravity vector (assumes fixed base)
fbk = group.getNextFeedbackFull();
baseRotMat = HebiUtils.quat2rotMat( [ fbk.orientationW(1), ...
                                      fbk.orientationX(1), ...
                                      fbk.orientationY(1), ...
                                      fbk.orientationZ(1) ] );
gravityVec = -baseRotMat(3,1:3);

end

