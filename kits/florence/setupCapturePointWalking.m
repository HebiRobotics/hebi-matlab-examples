% Capture point walking step parameters
function [params] = setupCapturePointWalking( )

    %%%%%%%%%%%%%%%%%%%
    % STEP PARAMETERS %
    %%%%%%%%%%%%%%%%%%%
    params.alpha = 0.03;  % lateral distance from foot at step ap ex
    params.sigma = 0.10;  % max saggital velocity at step apex
    params.delta = 0.10;  % min support exchange location
    params.omega = 0.20;  % max support exchange location

    params.halfFootLength = 0.075;  % max lever arm for ZMP
    params.halfFootWidth = 0.050;  % max lever arm for ZMP

    params.minStepTime = 0.33;  % used if we need to step 'immediately'.
    
    params.stepHeight = 0.050; % How high to lift the foot mid-swing
    params.swingPhaseTiming = [0 0.5 1];
end