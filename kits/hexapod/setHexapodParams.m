

% Robot Parameters
robotMass = 10;  % kg
robotWeight = 9.8 * robotMass;
numLegs = length(kin);
allLegs = 1:numLegs;

% Leg Indices
jointInds = 1:(3*numLegs);
jointInds = reshape(jointInds,3,6)';

% Stance Parameters
bodyHeight = .12; % meters
stanceRadius = .43;  % Stance Compliance parameters acts as a position gain


for leg=1:numLegs
    baseFrame = kin{leg}.getBaseFrame();
    homeStanceXYZ(:,leg) = baseFrame(1:3,1:3) * ...
                        [stanceRadius; 0; -bodyHeight];
end

levelHomeStanceXYZ = homeStanceXYZ;

% Step Parameters
stepHeight = .040; % meters (.040 default)
stepOverShoot = .3; % factor of step to overshoot 
stepPeriod = 0.7;  % seconds (.7 default)
stepPhase = [0 .5 1];

isStance = ones(1,6)==true;  % Boolean mask to determine stance feet
stepping = false;   % track state of stepping
legStepState = 0;   % Alternates set A and B in tripod gait

% Thresholds: sets how far COM can be from 'home' position before stepping
shiftThresh = .02; % meters (.025 default)
rotThresh = .05; % rad .1

