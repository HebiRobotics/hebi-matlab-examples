function checkIpadConnect(hasNewPhoneFbk)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

% If you lose connection to the iPad, stop all velocity commands
controllerLost = false;
controllerTextSent = false;
phoneFbkCounter = 0;

if ~hasNewPhoneFbk
    if phoneFbkCounter < 20
        phoneFbkCounter = phoneFbkCounter + 1;
        pause(0.01);
    else
        controllerLost = true;
        pause(0.01);
        leftPadLeftRight = 0;
        leftPadUpDown = 0;
        rightPadLeftRight = 0;
        rightPadUpDown = 0;

    end
else
    phoneFbkCounter = 0;
    controllerLost = false;
end

if controllerLost && ~controllerTextSent
    disp('Lost connection to Controller. Please reconnect.')
    robotGroup.send('led','b');
    controllerTextSent = true;
end

if ~controllerLost && controllerTextSent
    disp('Controller reconnected, demo continued.')
    robotGroup.send('led',[]);
    controllerTextSent = false;
end


end

