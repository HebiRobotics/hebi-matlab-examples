% Do some standard plots to check position, velocity, effort (torque) 
% tracking.
%
% Dave Rollinson
% Aug 2016

function [] = plotLogCommands(hebilog, group)

if nargin < 2
    names = [];
else
    names = group.getInfo().name;
end


% Position
figure(201);

ax = subplot(2,1,1);
plot(hebilog.time,hebilog.position);
ax.ColorOrderIndex = 1;
hold on;
plot(hebilog.time,hebilog.positionCmd,'--');
hold off;
title('Position');
xlabel('time (sec)');
ylabel('position (rad)');
legend(names);

subplot(2,1,2);
plot(hebilog.time,hebilog.position-hebilog.positionCmd);
title('Position Error');
xlabel('time (sec)');
ylabel('error (rad)');
ylim(ax.YLim);
legend(names);

% Velocity
figure(202);

ax = subplot(2,1,1);
plot(hebilog.time,hebilog.velocity);
ax = gca;
ax.ColorOrderIndex = 1;
hold on;
plot(hebilog.time,hebilog.velocityCmd,'--');
hold off;
title('Velocity');
xlabel('time (sec)');
ylabel('velocity (rad/sec)');
legend(names);

subplot(2,1,2);
plot(hebilog.time,hebilog.velocity-hebilog.velocityCmd);
title('Velocity Error');
xlabel('time (sec)');
ylabel('error (rad/sec)');
ylim(ax.YLim);
legend(names);


% Effort
figure(203);

ax = subplot(2,1,1);
plot(hebilog.time,hebilog.effort);
ax = gca;
ax.ColorOrderIndex = 1;
hold on;
plot(hebilog.time,hebilog.effortCmd,'--');
hold off;
title('Effort');
xlabel('time (sec)');
ylabel('effort (N-m)');
legend(names);

subplot(2,1,2);
plot(hebilog.time,hebilog.effort-hebilog.effortCmd);
title('Effort Error');
xlabel('time (sec)');
ylabel('error (N-m)');
ylim(ax.YLim);
legend(names);

end
