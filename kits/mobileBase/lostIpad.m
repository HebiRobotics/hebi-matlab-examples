function lostIpad()
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

hasNewPhoneFbk = ~isempty(phoneGroup.getNextFeedback( ...
            fbkPhoneIO, fbkPhoneMobile, ... % overwrite existing structs
            'timeout', 0 )); % prevent blocking due to bad comms
        
        hasNewRobotFbk = ~isempty(robotGroup.getNextFeedback( ...
            robotFbk, 'View', 'Full', 'timeout', 0 ));
controllerLost = true;
pause(0.01);
leftPadLeftRight = 0; % Right Pad Up/Down
leftPadUpDown = 0;
rightPadLeftRight = 0;
rightPadUpDown = 0;

if controllerLost && ~controllerTextSent
    disp('Lost connection to Controller. Please reconnect.')
    robotGroup.send('led','b');
    controllerTextSent = true;
end

end

