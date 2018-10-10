function [] = scanningArmAnalysis( log )
    %%
    % Load a log from scanningArmRaster and plot stuff
    %
    % Dave Rollinson
    % Jul 2018

    kin = HebiKinematics('scanningArm_4DoF');
    if nargin < 1
        disp('  No log given, please choose a log file...');
        log = HebiUtils.loadGroupLogsUI('view','full');
    end
    log = struct(log);

    %%
    % Plotting XYZ tracking
    logLength = length(log.time);
    cmdXYZ_mm = nan(logLength,3);
    fbkXYZ_mm = nan(logLength,3); 

    % Calculate kinematics, converting positions to [mm]
    for i = 1:length(log.time)
        tempFrame = kin.getFK('endeffector',log.positionCmd(i,:));
        cmdXYZ_mm(i,:) = 1000 * tempFrame(1:3,4);
        tempFrame = kin.getFK('endeffector',log.position(i,:));
        fbkXYZ_mm(i,:) = 1000 * tempFrame(1:3,4);
    end

    %%
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

    figure(202);
    plot(log.time, fbkXYZ_mm(:,1) - cmdXYZ_mm(:,1), 'r');
    hold on;
    plot(log.time, fbkXYZ_mm(:,2) - cmdXYZ_mm(:,2), 'g');
    plot(log.time, fbkXYZ_mm(:,3) - cmdXYZ_mm(:,3), 'b');
    hold off;

    title('End-Effector Error - Time');
    xlabel('time (sec)');
    ylabel('error (mm)');
    grid on;
    xlim([0 log.time(end)]);
    legend x y z;


    figure(203);
    plot(cmdXYZ_mm(:,1), fbkXYZ_mm(:,1) - cmdXYZ_mm(:,1), 'r.');
    hold on;
    plot(cmdXYZ_mm(:,1), fbkXYZ_mm(:,2) - cmdXYZ_mm(:,2), 'g.');
    plot(cmdXYZ_mm(:,1), fbkXYZ_mm(:,3) - cmdXYZ_mm(:,3), 'b.');
    hold off;

    title('End-Effector Error - Along Raster');
    xlabel('x (mm)');
    ylabel('error (mm)');
    axis equal;
    grid on;
    legend x y z;

end

