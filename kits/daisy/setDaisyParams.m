% Robot-Specific Parameters
% This is called in runHexapod.m.  In the future it would be good to
% combine all the setup into a function, setupDaisy.m, like we do with the
% Rosie and Florence kits.
%
% Dave Rollinson
% Apr 2019

% Masses / Weights.  Assume all the legs weigh the same
legMass = sum( legKin{1}.getBodyMasses() );
chassisMass = sum(chassisKin.getBodyMasses());
robotMass = chassisMass + 6*legMass;  % kg
robotWeight = 9.8 * robotMass;

numLegs = length(legKin);
allLegs = 1:numLegs;

% Stance Parameters
bodyHeight = .21; % meters
stanceRadius = .55;  % meters
for leg=1:numLegs
    baseFrame = legKin{leg}.getFirstJointFrame();
    homeStanceXYZ(:,leg) = baseFrame(1:3,1:3) * ...
                        [stanceRadius; 0; -bodyHeight];
end

levelHomeStanceXYZ = homeStanceXYZ;

% Step Parameters
stepHeight = .040; % meters (.040 default)
stepOverShoot = 0.35; % factor of step to overshoot 
stepPeriod = 0.7;  % seconds (.7 default)
stepPhase = [0 .5 1];

isStance = ones(1,6)==true;  % Boolean mask to determine stance feet
stepping = false;   % track state of stepping
legStepState = 0;   % Alternates set A and B in tripod gait

% Thresholds: sets how far COM can be from 'home' position before stepping
shiftThresh = .02; % meters (.025 default)
rotThresh = .05; % rad .1

