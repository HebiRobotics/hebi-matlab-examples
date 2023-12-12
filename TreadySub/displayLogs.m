%show logs

% logs = HebiUtils.loadGroupLogsUI();

leaderEffort = leaderLog.effort;
[m,n] = size(leaderEffort);
followEffort = followLog.effort(1:m,1:n);
diff = leaderEffort - followEffort;
HebiUtils.plotLogs( leaderLog, 'effort', 'fignum', 101);
HebiUtils.plotLogs( followLog, 'effort', 'fignum', 102);
figure(3);
plot( 1:length(diff),diff);
% HebiUtils.plotLogs( logs, 'effort', 'fignum', 103);
% HebiUtils.plotLogs( logs, 'effortCmd', 'fignum', 104);
% HebiUtils.plotLogs( logs, 'motorCurrent', 'fignum', 105);