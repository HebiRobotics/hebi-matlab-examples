function [] = kinematics_analysis( log, kin )
%%
% Use a log file and kinematic info for an arm and plot end-effector
% motion and tracking error.
%
% Dave Rollinson
% Jul 2018

% Convert log file to struct to speed up reading.
log = struct(log);
    
logLength = length(log.time);
cmdXYZ = nan(logLength,3);
fbkXYZ = nan(logLength,3); 
cmdVel_6DoF = nan(logLength,6);
fbkVel_6DoF = nan(logLength,6); 

%%
% XYZ position tracking

% Calculate kinematics for commanded and feedback
for i = 1:length(log.time)
    tempFrame = kin.getFK('endeffector',log.positionCmd(i,:));
    cmdXYZ(i,:) = tempFrame(1:3,4);
    tempFrame = kin.getFK('endeffector',log.position(i,:));
    fbkXYZ(i,:) = tempFrame(1:3,4);
end

% Calculate XYZ position Error
errorXYZ = fbkXYZ - cmdXYZ;
errorNorm = sqrt( errorXYZ(:,1).^2 + ...
                  errorXYZ(:,2).^2 + ...
                  errorXYZ(:,3).^2 );
                 
%%
% XYZ velocity tracking

% Calculate Jacobians for commanded and feedback
for i = 1:length(log.time)
    J_cmd(:,:,i) = kin.getJacobian( 'endeffector', log.positionCmd(i,:) );
    J_fbk(:,:,i) = kin.getJacobian( 'endeffector', log.position(i,:) );
    
    cmdVel_6DoF(i,:) = J_fbk(:,:,i) * log.velocityCmd(i,:)';
    fbkVel_6DoF(i,:) = J_fbk(:,:,i) * log.velocity(i,:)';
end

% Calculate XYZ velocity Error
errorVel_6DoF = fbkVel_6DoF - cmdVel_6DoF;
errorVelNorm = sqrt( errorVel_6DoF(:,1).^2 + ...
                     errorVel_6DoF(:,2).^2 + ...
                     errorVel_6DoF(:,3).^2 );
                 
                 
   
%%
% Plotting

fbkXYZ_mm = 1000 * fbkXYZ;
cmdXYZ_mm = 1000 * cmdXYZ;
errorXYZ_mm = 1000 * errorXYZ;
errorNorm_mm = 1000 * errorNorm;

figure();
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

% If there were any commanded positions, plot the tracking error
if any(~isnan(cmdXYZ(:)))
    figure();
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



