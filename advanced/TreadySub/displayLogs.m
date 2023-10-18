%show logs

logs = HebiUtils.loadGroupLogsUI();

HebiUtils.plotLogs( logs, 'position', 'fignum', 101);
HebiUtils.plotLogs( logs, 'velocity', 'fignum', 102);
HebiUtils.plotLogs( logs, 'effort', 'fignum', 103);
HebiUtils.plotLogs( logs, 'effortCmd', 'fignum', 104);
% HebiUtils.plotLogs( logs, 'motorCurrent', 'fignum', 105);