% Load and plot logs from Lily
% Dave Rollinson
% Oct 2020

clear *;
%close all;

logFileName = 'logs/Lily_5kg_payload_mass_29kg_10-19-2020.hebilog';
%logFileName = 'logs/Lily_no_payload_mass_24kg_10-19-2020.hebilog';

[log, info, gains] = HebiUtils.loadGroupLog( logFileName, 'view', 'full' );

baseMask = 1:3:18;
shoulderMask = 2:3:18;
elbowMask = 3:3:18;

plotMask = shoulderMask;

HebiUtils.plotLogs( log, 'position', 'modules', plotMask );
HebiUtils.plotLogs( log, 'velocity', 'modules', plotMask );
HebiUtils.plotLogs( log, 'effort', 'modules', plotMask );

% HebiUtils.plotLogs( log, 'windingTemperature', 'modules', plotMask );
% HebiUtils.plotLogs( log, 'motorTemperature', 'modules', plotMask );