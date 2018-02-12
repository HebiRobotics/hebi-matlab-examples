function [ group, kin, gravityVec ] = setupArm_beverages()

% NOTE: CHANGE AS NEEDED TO MATCH YOUR CONFIGURATION

% Communications
group = HebiLookup.newGroupFromNames('beverageArm', {
   'Base'
   'Shoulder'
   'Elbow'
   'Wrist1'
   'Wrist2'
});

% Kinematics
kin = HebiKinematics();
kin.addBody('X5-4');
kin.addBody('X5-HeavyBracket', 'mount', 'right-outside');
kin.addBody('X5-9');
kin.addBody('X5-Link', 'extension', .29, 'twist', pi);
kin.addBody('X5-9');
kin.addBody('X5-Link', 'extension', .29, 'twist', pi, 'mass', .500);
kin.addBody('X5-4');
kin.addBody('X5-LightBracket', 'mount', 'right');
kin.addBody('X5-1');
kin.addBody('GenericLink', 'CoM', [0 0 .02], 'Output', eye(4), 'Mass', .150 );

% Determine gravity vector (assumes fixed base)
fbk = group.getNextFeedback();
gravityVec = -[0 0 1];

end

