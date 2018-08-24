% Test out encoder tics on I/O board
%
% Dave Rollinson
% July 2018

clear *;
close all;

group = HebiLookup.newGroupFromNames('*','curtis_test');
group.setFeedbackFrequency(500);
cmd = IoCommandStruct();

setX = 'e1';
setY = 'e3';

fbkX = 'c1';
fbkY = 'c4';

group.startLog();

cmd.(setX) = 1;
cmd.(setY) = 1;
group.send(cmd);

pause(1.0);

cmd.(setX) = 100;
cmd.(setY) = 100;
group.send(cmd);

pause(1.0);

cmd.(setX) = 100;
cmd.(setY) = 100;
group.send(cmd);

pause(1.0);

cmd.(setX) = 1;
cmd.(setY) = 1;
group.send(cmd);

pause(1.0);

log = group.stopLogIO();

%%
figure(101);
plot(log.time,log.(fbkX));
hold on;
plot(log.time,log.(fbkY));
hold off;
xlabel('time (sec)');
ylabel('ticks');
title('Spoofed Encoder Ticks');