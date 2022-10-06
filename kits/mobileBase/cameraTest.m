HebiLookup.initialize();

% Optional step to limit the lookup to a set of interfaces or modules
% HebiLookup.setLookupAddresses('10.10.10.255');

robotFamily = 'Chevron';
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % Setup Camera %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

camGroup = HebiLookup.newGroupFromNames( robotFamily, 'Widey' );
camCmdIO = IoCommandStruct();

LED = 'f2';
ledScale = 0.4;
ledMin = 0.30;
ledMax = 0.90;

ledDimmerLast = -0.75;
ledControlScale = 0;
ledButtonLast = 0;
ledHighlight = 0;
camHomeButtonLast = 0;

camCmdIO.(LED) = [ledMin];

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Setup Mobile Phone Input %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Button Mapping
flipperSlider1 = 'a3';
flipperSlider2 = 'a4';
flipperSlider3 = 'a5';
flipperSlider4 = 'a6';
resetPoseButton = 'b1';
LEDButton = 'b3';
LEDSlider = 'a3';
camHomeMode = 'b6';
quitDemoButton = 'b8';
probeXAxis = 'a1'; % Right Pad Up/Down
probeYAxis = 'a2'; % Right Pad Left/Right
xVelAxis = 'a8'; % Right Pad Up/Down
yVelAxis = 'a7'; % Right Pad Left/Right
% rotVelAxis = 'a1'; % Left Pad Left/Right


camVelScale = 1; % rad/sec

% Search for phone controller. Allow retry because phones tend to
% drop out when they aren't used (i.e. sleep mode)
phoneName = 'mobileIO';
while true
    try
        fprintf('Searching for phone Controller...\n');
        phoneGroup = HebiLookup.newGroupFromNames(robotFamily, phoneName);
        disp('Phone Found.  Starting up');
        break;
    catch
        pause(1.0);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%
% Camera / LED Controls %
%%%%%%%%%%%%%%%%%%%%%%%%%
phoneButtonGroup = HebiMobileIO.findDevice(robotFamily, phoneName);
ledDimmer = (fbkPhoneIO.(LEDSlider) + 1.0) / 2;
if ledDimmer < ledMin
    ledDimmer = 0;
elseif ledDimmer > ledMax
    ledDimmer = ledMax;
end

if (fbkPhoneIO.(LEDButton) - ledButtonLast) == 1
    ledControlScale = ~ledControlScale;
    ledHighlight = ~ledHighlight;
    phoneButtonGroup.setButtonIndicator(5, ledHighlight);
end

ledButtonLast = fbkPhoneIO.(LEDButton);
LEDCmd = ledControlScale * ledDimmer;

camCmdIO.(LED) = [LEDCmd];
camCmdIO.(LED) = min( camCmdIO.(LED), ledMax );

camGroup.send(camCmdIO);

