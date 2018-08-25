function [] = plotEndEffectorError( log, kin, group )

    % Kinematic Error
    legLength = length(log.time);
    xyzCmd = nan(legLength,3);
    xyzFbk = nan(legLength,3);

    for i=1:legLength

        tipFrameFbk = kin.getFK('endEffector',log.position(i,:));
        tipFrameCmd = kin.getFK('endEffector',log.positionCmd(i,:));

        xyzFbk(i,:) = tipFrameFbk(1:3,4);
        xyzCmd(i,:) = tipFrameCmd(1:3,4);
    end

    xyzFbk_mm = xyzFbk * 1000;
    xyzCmd_mm = xyzCmd * 1000;

    xyzError_mm = xyzFbk_mm - xyzCmd_mm;

    %% Plotting

    % 3D Tracking
    figure(101);
    plot3(xyzFbk_mm(:,1),xyzFbk_mm(:,2),xyzFbk_mm(:,3));
    hold on;
    plot3(xyzCmd_mm(:,1),xyzCmd_mm(:,2),xyzCmd_mm(:,3));
    hold off;
    legend('Feedback','Command');
    title('End Effector Tracking');
    xlabel('x (mm)');
    ylabel('y (mm)');
    zlabel('z (mm)');
    axis equal;
    grid on;

    % End Effector Error
    figure(102);
    subplot(2,1,1);
    plot(log.time, xyzError_mm);
    legend('x','y','z');
    title('Component End Effector Error');
    xlabel('time (sec)');
    ylabel('error (mm)');
    grid on;

    subplot(2,1,2);
    plot(log.time, sqrt(sum(xyzError_mm.^2,2)),'k');
    title('Total Effector Error');
    xlabel('time (sec)');
    ylabel('error (mm)');
    grid on;
    
end



