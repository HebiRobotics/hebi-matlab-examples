% Load the saved data from an inspection log and plot
%
% Dave Rollinson
% May 2018

logFileName = 'logs/scanLog_020mm.hebilog';

%% 
% Load the data
log = HebiUtils.loadGroupLog( logFileName, 'view', 'full' );
log = struct( log );  % Convert to Matlab struct to speed things up later

if ~exist('kin','var')
    [~, kin, gravityVec] = setupArm_elbowScanner();
end

group = HebiUtils.newGroupFromLog( logFileName );
groupInfo = group.getInfo();
while isempty(groupInfo)
    fbk = group.getNextFeedback();
    groupInfo = group.getInfo();
end

%% 
% Kinematic Error
plotEndEffectorError(log,kin,group);


% System Power
basePower = (24 * .075);
motorPower = log.voltage .* log.motorCurrent;

figure(104);
subplot(2,1,1);
plot(log.time, motorPower + basePower);
title('Power Draw Per Joint');
xlabel('time (sec)');
ylabel('power (W)');
xlim([0 log.time(end)]);
ylim([0 100]);
legend(group.getInfo.name);
grid on;

subplot(2,1,2);
plot(log.time, sum(motorPower,2) + group.getNumModules*basePower, 'k' );
title('Total Power Draw');
xlabel('time (sec)');
ylabel('power (W)');
xlim([0 log.time(end)]);
ylim([0 100]);
grid on;




