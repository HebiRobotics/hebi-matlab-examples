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
legLength = length(log.time);
xyzCmd = nan(legLength,3);
xyzFbk = nan(legLength,3);
xyzVelCmd = nan(legLength,3);
xyzVelFbk = nan(legLength,3);

for i=1:legLength
    
    tipFrameFbk = kin.getFK('endEffector',log.position(i,:));
    tipFrameCmd = kin.getFK('endEffector',log.positionCmd(i,:));
    
    tipJ_fbk = kin.getJacobian('endEffector',log.position(i,:));
    tipJ_cmd = kin.getJacobian('endEffector',log.positionCmd(i,:));
    
    xyzFbk(i,:) = tipFrameFbk(1:3,4);
    xyzCmd(i,:) = tipFrameCmd(1:3,4);
    
    velFbk = tipJ_fbk * log.velocity(i,:)';
    velCmd = tipJ_cmd * log.velocityCmd(i,:)';
    
    xyzVelFbk(i,:) = velFbk(1:3);
    xyzVelCmd(i,:) = velCmd(1:3);
end

xyzFbk_mm = xyzFbk * 1000;
xyzCmd_mm = xyzCmd * 1000;

xyzVelFbk_mm = xyzVelFbk * 1000;
xyzVelCmd_mm = xyzVelCmd * 1000;

xyzError_mm = xyzFbk_mm - xyzCmd_mm;

%% Plotting

% 3D Tracking
figure(101);
plot3(xyzFbk_mm(:,1),xyzFbk_mm(:,2),xyzFbk_mm(:,3));
hold on;
plot3(xyzCmd_mm(:,1),xyzCmd_mm(:,2),xyzCmd_mm(:,3),':');
hold off;
legend('Feedback','Command');
title('End Effector Tracking');
xlabel('x (mm)');
ylabel('y (mm)');
zlabel('z (mm)');
axis equal;
grid on;

% End Effector Position Error
figure(102);
subplot(2,1,1);
plot(log.time, xyzError_mm);
legend('x','y','z');
title('Component End Effector Error');
xlabel('time (sec)');
ylabel('error (mm)');
xlim([0 log.time(end)]);
grid on;

subplot(2,1,2);
plot(log.time, sqrt(sum(xyzError_mm.^2,2)),'k');
title('Total Effector Error');
xlabel('time (sec)');
ylabel('error (mm)');
xlim([0 log.time(end)]);
grid on;

% End Effector Velocity
figure(103);
subplot(2,1,1);
plot(log.time, xyzVelFbk_mm);
legend('x','y','z');
title('Component End Effector Velocity');
xlabel('time (sec)');
ylabel('velocity (mm/sec)');
xlim([0 log.time(end)]);
grid on;

subplot(2,1,2);
plot(log.time, sqrt(sum(xyzVelFbk_mm.^2,2)),'k');
title('Total Effector Speed');
xlabel('time (sec)');
ylabel('speed (mm/sec)');
xlim([0 log.time(end)]);
grid on;


% End Effector Velocity
basePower = (36 * .05);
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




