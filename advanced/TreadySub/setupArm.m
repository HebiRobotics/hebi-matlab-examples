function [ group, group2, arm, kin, gravityVec ] = setupArm()

% NOTE: CHANGE AS NEEDED TO MATCH YOUR CONFIGURATION

% Communications
group = HebiLookup.newGroupFromNames('Dow-Leader', {
   'J1'
   'J2L'
%   'J2R'
   'J3'
   'J4'
   'J5'
   'J6'
   'J7'
});

group2 = HebiLookup.newGroupFromNames('Dow-Leader', {
    'J2R'
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
kin = HebiUtils.loadHRDF('./R-Series-7DOF.hrdf');

% Arm setup
arm = HebiArm(group, kin);
% arm.plugins = {HebiArmPlugins.DoubledJointMirror(2, group2)};

% Determine gravity vector (assumes fixed base)
fbk = group.getNextFeedbackFull();
baseRotMat = HebiUtils.quat2rotMat( [ fbk.orientationW(1), ...
                                      fbk.orientationX(1), ...
                                      fbk.orientationY(1), ...
                                      fbk.orientationZ(1) ] );
gravityVec = -baseRotMat(3,1:3);

end

