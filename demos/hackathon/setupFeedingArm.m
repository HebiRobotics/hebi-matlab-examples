function [ group, kin, gravityVec ] = setupFeedingArm()

% NOTE: CHANGE AS NEEDED TO MATCH YOUR CONFIGURATION

% Communications
group = HebiLookup.newGroupFromNames( 'Arm', { 'Base'
                                               'Shoulder'
                                               'Elbow'
                                               'Wrist1'
                                               'Wrist2' } );

gains = HebiUtils.loadGains('defaultFeedingGains.xml');
group.send('gains',gains);

% End Effector Kinematics
xyz_endEff = [.150 0 .01]; % Fork mounted in a standard tube
rot_endEff = R_y(pi/2);

% 4x4 Homogeneous transform of end effector output frame
output_EndEff = eye(4);
output_EndEff(1:3,4) = xyz_endEff;
output_EndEff(1:3,1:3) = rot_endEff;

comEndEff = xyz_endEff / 2;
massEndEff = .2; % kg   .2 default

% Arm Kinematics
kin = HebiKinematics();
kin.addBody('X5-4');
kin.addBody('X5-HeavyBracket', 'mount', 'right-outside');
kin.addBody('X5-9');
kin.addBody('X5-Link', 'extension', .325, 'twist', pi, 'mass', .280);
kin.addBody('X5-9');
kin.addBody('X5-Link', 'extension', .325, 'twist', pi, 'mass', .280);
kin.addBody('X5-1');
kin.addBody('X5-LightBracket', 'mount', 'right');
kin.addBody('X5-1');
kin.addBody('GenericLink', 'com', comEndEff, ...
                           'out', output_EndEff, ...
                           'mass', massEndEff );
                       
baseFrame = eye(4);
baseFrame(1:3,1:3) = R_z(pi);
kin.setBaseFrame( baseFrame );  

% Determine gravity vector (assumes fixed base frame)
fbk = group.getNextFeedback();
gravityVec = -[fbk.accelX(1); fbk.accelY(1); fbk.accelZ(1)];
baseFrame = kin.getBaseFrame;
gravityVec = baseFrame(1:3,1:3) * gravityVec;


end

