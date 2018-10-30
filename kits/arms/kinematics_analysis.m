function [] = kinematics_analysis( log, kin )
%%
% Use a log file and kinematic info for an arm and plot end-effector
% motion and tracking error.
%
% Dave Rollinson
% Jul 2018

% Convert log file to struct to speed up reading.
log = struct(log);
    
%%
% XYZ tracking
logLength = length(log.time);
cmdXYZ_mm = nan(logLength,3);
fbkXYZ_mm = nan(logLength,3); 

% Calculate kinematics for commanded and feedback, 
% converting positions to [mm]
for i = 1:length(log.time)
    tempFrame = kin.getFK('endeffector',log.positionCmd(i,:));
    cmdXYZ_mm(i,:) = 1000 * tempFrame(1:3,4);
    tempFrame = kin.getFK('endeffector',log.position(i,:));
    fbkXYZ_mm(i,:) = 1000 * tempFrame(1:3,4);
end

% Calculate Error
errorXYZ_mm = fbkXYZ_mm - cmdXYZ_mm;
errorNorm_mm = sqrt( errorXYZ_mm(:,1).^2 + ...
                     errorXYZ_mm(:,2).^2 + ...
                     errorXYZ_mm(:,3).^2 );
   
%%
% Plotting
figure(201);
plot3( fbkXYZ_mm(:,1), fbkXYZ_mm(:,2), fbkXYZ_mm(:,3), 'b' );
hold on;
plot3( cmdXYZ_mm(:,1), cmdXYZ_mm(:,2), cmdXYZ_mm(:,3), 'r:' );
hold off;

title('End-Effector Tracking');
xlabel('x (mm)');
ylabel('y (mm)');
zlabel('z (mm)');
axis equal;
grid on;
legend feedback command;

% If there were commanded positions, plot the tracking error
if all(~isnan(cmdXYZ_mm(:)))
    figure(202);
    subplot(2,1,1);
    plot(log.time, errorXYZ_mm(:,1), 'r');
    hold on;
    plot(log.time, errorXYZ_mm(:,2), 'g');
    plot(log.time, errorXYZ_mm(:,3), 'b');
    hold off;

    title('End-Effector XYZ Error - Time');
    xlabel('time (sec)');
    ylabel('error (mm)');
    grid on;
    xlim([0 log.time(end)]);
    legend x y z;

    subplot(2,1,2);
    plot(log.time, errorNorm_mm, 'k');
    title('End-Effector Total Error - Time');
    xlabel('time (sec)');
    ylabel('error (mm)');
    grid on;
    xlim([0 log.time(end)]);        
end



