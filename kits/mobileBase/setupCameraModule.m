function [ arm, params] = setupPanTiltArm_Chevron(robotfamily)

%%%%%%%%%%%%%%%%
% Setup Cameras %
%%%%%%%%%%%%%%%%

camGroup = HebiLookup.newGroupFromNames( robotFamily, {'C10-0003'} );
camCmdIO = IoCommandStruct();

cameraZoom = 'f1';
cameraZoomScale = 0.1;
zoomMin = 0.00;
zoomMax = 1.0;

spotLED = 'f2';
floodLED = 'f3';
ledScale = 0.4;
ledMin = 0.30;
ledMax = 0.90;

ledDimmerLast = -0.75;
tailLedDimmerLast = -0.75;
camZoomSliderLast = -0.75;

tailSpotControlScale = 0;
tailFloodControlScale = 0;

tailSpotButtonLast = 0;
tailFloodButtonLast = 0;


tailSpotHighlight = 0;
tailFloodHighlight = 0;

camHomeButtonLast = 0;

camCmdIO.(cameraZoom) = [nan nan 0.0];
camCmdIO.(spotLED) = [ledMin ledMin ledMin];
camCmdIO.(floodLED) = [nan nan ledMin];


cameraRotOffset = 0;   % [deg]
RPY = [0 0 0];

end