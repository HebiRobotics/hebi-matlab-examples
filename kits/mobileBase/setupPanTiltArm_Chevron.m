function [panTiltArm, panTiltParams] = setupPanTiltArm_Chevron(robotFamily)


% Arm Module Names
params.panTiltNames(family, {
    'C1_pan'
    'C2_tilt' });

% % Load and send Gains
% params.gains = HebiUtils.loadGains('gains/6-dof-arm-gains-rosie');
% HebiUtils.sendWithRetry(group, 'gains', params.gains)

%Setup camera tail kinematics
camKin = HebiKinematics('hrdf/Chevron-CamTail');

params.armDoF = [5 6];

% API Wrappers
panTiltArm = HebiArm(group, camKin);


end