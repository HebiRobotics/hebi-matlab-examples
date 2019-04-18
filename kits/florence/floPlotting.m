% Plotting after running the Florence Code
%
% Dave Rollinson
% Mar 2019

actuatorNames = params.actuatorNames;
legIndex = params.legIndex;

if logging
    figureSeries = 100;

    HebiUtils.plotLogs(log,'position','modules',legIndex{1},'figNum',figureSeries+11);
    legend(actuatorNames{legIndex{1}});
    HebiUtils.plotLogs(log,'position','modules',legIndex{2},'figNum',figureSeries+12);
    legend(actuatorNames{legIndex{2}});

    HebiUtils.plotLogs(log,'velocity','modules',legIndex{1},'figNum',figureSeries+21);
    legend(actuatorNames{legIndex{1}});
    HebiUtils.plotLogs(log,'velocity','modules',legIndex{2},'figNum',figureSeries+22);
    legend(actuatorNames{legIndex{2}});

    HebiUtils.plotLogs(log,'effort','modules',legIndex{1},'figNum',figureSeries+31);
    ylim([-5 5]);
    legend(actuatorNames{legIndex{1}});
    HebiUtils.plotLogs(log,'effort','modules',legIndex{2},'figNum',figureSeries+32);
    ylim([-5 5]);
    legend(actuatorNames{legIndex{2}});
end
    
%%
%%%%%%%%%%%%%%%%%%%%%%%%%
% IMPEDANCE CONTROLLERS %
%%%%%%%%%%%%%%%%%%%%%%%%%
figureSeries = 200;

figure(figureSeries+1);
subplot(2,1,1);
plot(timeHist,squeeze(footIkWrenchHist(:,1,:)));
title('Foot IK Wrench - Left Leg');
xlabel('time (sec)');
xlim([0 timeHist(end)]);
ylabel('wrench force (N or N-m)');
legend('x','y','z','rx','ry','rz');
grid on;
subplot(2,1,2);
plot(timeHist,squeeze(footIkWrenchHist(:,2,:)));
title('Foot IK Wrench - Right Leg');
xlabel('time (sec)');
xlim([0 timeHist(end)]);
ylabel('wrench force (N or N-m)');
grid on;

figure(figureSeries+2);
subplot(2,1,1);
plot(timeHist,squeeze(chassisWrenchHist(:,1,:)));
title('Chassis Wrench - Left Leg');
xlabel('time (sec)');
xlim([0 timeHist(end)]);
ylabel('wrench force (N or N-m)');
legend('x','y','z','rx','ry','rz');
grid on;
subplot(2,1,2);
plot(timeHist,squeeze(chassisWrenchHist(:,2,:)));
title('Chassis Wrench - Right Leg');
xlabel('time (sec)');
xlim([0 timeHist(end)]);
ylabel('wrench force (N or N-m)');
grid on;

figure(figureSeries+3);
subplot(2,1,1);
plot(timeHist,squeeze(stanceWrench1Hist(:,1,:)));
title('One-Leg Stance Wrench - Left Leg');
xlabel('time (sec)');
xlim([0 timeHist(end)]);
ylabel('wrench force (N or N-m)');
legend('x','y','z','rx','ry','rz');
grid on;
subplot(2,1,2);
plot(timeHist,squeeze(stanceWrench1Hist(:,2,:)));
title('One-Leg Stance Wrench - Right Leg');
xlabel('time (sec)');
xlim([0 timeHist(end)]);
ylabel('wrench force (N or N-m)');
grid on;

figure(figureSeries+4);
subplot(2,1,1);
plot(timeHist,squeeze(stanceWrench2Hist(:,1,:)));
title('Two-Leg Stance Wrench - Left Leg');
xlabel('time (sec)');
xlim([0 timeHist(end)]);
ylabel('wrench force (N or N-m)');
legend('x','y','z','rx','ry','rz');
grid on;
subplot(2,1,2);
plot(timeHist,squeeze(stanceWrench2Hist(:,2,:)));
title('Two-Leg Stance Wrench - Right Leg');
xlabel('time (sec)');
xlim([0 timeHist(end)]);
ylabel('wrench force (N or N-m)');
grid on;


    
% %%%%%%%%%%%%%
% % POSITIONS %
% %%%%%%%%%%%%%
% figure(figureSeries+1);
% 
% subplot(2,1,1);
% plot(timeHist,angHist(:,leftLeg));
% grid on;
% xlim([0 timeHist(end)]);
% ylim([-4 4]);
% title('Positions - Left Leg');
% xlabel('time (sec)');
% ylabel('position (rad)');
% legend(jointNames);
% 
% subplot(2,1,2);
% plot(timeHist,angHist(:,rightLeg));
% grid on;
% xlim([0 timeHist(end)]);
% ylim([-4 4]);
% title('Positions - Right Leg');
% xlabel('time (sec)');
% ylabel('position (rad)');
% legend(jointNames);
% 
% 
% %%%%%%%%%%%%%%
% % VELOCITIES %
% %%%%%%%%%%%%%%
% figure(figureSeries+2);
% 
% subplot(2,1,1);
% plot(timeHist,angVelHist(:,leftLeg));
% grid on;
% xlim([0 timeHist(end)]);
% ylim([-6 6]);
% title('Velocities - Left Leg');
% xlabel('time (sec)');
% ylabel('position (rad/sec)');
% legend(jointNames);
% 
% subplot(2,1,2);
% plot(timeHist,angVelHist(:,rightLeg));
% grid on;
% xlim([0 timeHist(end)]);
% ylim([-6 6]);
% title('Velocities - Right Leg');
% xlabel('time (sec)');
% ylabel('velocity (rad/sec)');
% legend(jointNames);
% 
% 
% %%%%%%%%%%%
% % EFFORTS %
% %%%%%%%%%%%
% figure(figureSeries+3);
% 
% subplot(2,1,1);
% plot(timeHist,effortHist(:,leftLeg));
% grid on;
% xlim([0 timeHist(end)]);
% ylim([-30 30]);
% title('Efforts - Left Leg');
% xlabel('time (sec)');
% ylabel('effort (N-m)');
% legend(jointNames);
% 
% subplot(2,1,2);
% plot(timeHist,effortHist(:,rightLeg));
% grid on;
% xlim([0 timeHist(end)]);
% ylim([-30 30]);
% title('Efforts - Right Leg');
% xlabel('time (sec)');
% ylabel('effort (N-m)');
% legend(jointNames);



