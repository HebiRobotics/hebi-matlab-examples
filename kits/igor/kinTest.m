% Setup a simple 3 dof arm made of X5 modules
kin = HebiKinematics();
kin.addBody('X5-4');
kin.addBody('X5Bracket');
kin.addBody('X5-4');
kin.addBody('X5Link', 'extension', 0.325, 'twist', pi/2);
kin.addBody('X5-1');
kin.addBody('X5Link', 'extension', 0.325, 'twist', 0);