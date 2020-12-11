function [footData] = getFootLoadCellData (group,sensorOffset)

% Read Feedback
fbk = group.getNextFeedbackIO();

% Convert load cell voltage output to force in lbs
% The load cell used is this one below:
% https://www.digikey.com/en/products/detail/te-connectivity-measurement-specialties/FC2231-0000-0100-L/809397

% sensorForceFrontLeft = (fbk.a5 - 0.525)/4 * 100;
% sensorForceFrontRight = (fbk.a8 - 0.514)/4 * 100;
% sensorForceBackLeft = (fbk.a6 - 0.507)/4 * 100;
% sensorForceBackRight = (fbk.a7 - 0.511)/4 * 100;

sensorForceFrontLeft = (fbk.a5 - sensorOffset(1))/4 * 100;
sensorForceFrontRight = (fbk.a8 - sensorOffset(2))/4 * 100;
sensorForceBackLeft = (fbk.a6 - sensorOffset(3))/4 * 100;
sensorForceBackRight = (fbk.a7 - sensorOffset(4))/4 * 100;

F = [sensorForceFrontLeft,sensorForceBackLeft,sensorForceBackRight,...
    sensorForceFrontRight];

F_threshold = 0.1; %artificial minimum force measurement in lbs
F = max(F,F_threshold); %set minimum threshold to ignore baseline noise

% Calculate Center of Pressure COP

l = 52; %x axis distance mm
w = 46; %y axis distance mm

X_COP = l *(-F(1)-F(2)+F(3)+F(4))/(F(1)+F(2)+F(3)+F(4));
Y_COP = w *(F(1)-F(2)-F(3)+F(4))/(F(1)+F(2)+F(3)+F(4));
    
COP = [X_COP,Y_COP];

% Set Structure
footData.F = F;
footData.COP = COP;
footData.group = group;

end
