% Testing out the pose filter API
%
% Dave Rollinson
% Mar 2017

clear *;
close all;

group = HebiLookup.newGroupFromNames('*',{'X-00188'});
fbkRate = group.setFeedbackFrequency(200);
pause(.1);
fbk = group.getNextFeedbackFull();
timeStart = fbk.hwTxTime;

% Keep initializing filter until it works
T_init = nan(4);
while any(isnan(T_init(:)))
    
    disp('Initializing filter.');
    fbk = group.getNextFeedbackFull();
    
    poseFilter1 = HebiPoseFilter();
    poseFilter1.setGyroScale( 1 );
    poseFilter1.setMaxAccelWeight( .01 );
    poseFilter1.setMaxAccelNormDev( .3 );
    
%     poseFilter2 = HebiPoseFilter();
%     poseFilter2.setGyroScale( 1E-6 );
%     poseFilter2.setMaxAccelWeight( 0.001 );
%     poseFilter2.setMaxAccelNormDev( .1 );

    accel = [fbk.accelX; fbk.accelY; fbk.accelZ];
    gyro = [fbk.gyroX; fbk.gyroY; fbk.gyroZ];
    filterTime = fbk.hwTxTime - timeStart;
    
    poseFilter1.update( accel(:,1), gyro(:,1), filterTime(1) );
    %poseFilter2.update( accel(:,1), gyro(:,1), filterTime(1) );
    
    poseFilter1.update( [0 0 10], [0 0 0], filterTime(1) );
    
    T_init(:,:,1) = poseFilter1.getPose();
%     T_init(:,:,2) = poseFilter2.getPose();
end
    
% poseFilterEKF = HebiPoseEKF();

% % Start the filter with 'clean' data to ensure it initializes well
% accel = [0.01; 0.01; 9.81];
% gyro = [0.01; 0.01; 0.01];



% poseFilter2.update( accel, gyro, hwTxTime(2) );

animStruct.fig = figure(101);
axisLen = .1*ones(3,1);
isFirstDraw = true;

filterTime = 0;
cmd = CommandStruct();

%group.startLog;
%tic;

while true
    fbk = group.getNextFeedbackFull();
%     dt = fbk.hwTxTime - filterTime
    filterTime = fbk.hwTxTime - timeStart;

    accel = [fbk.accelX; fbk.accelY; fbk.accelZ];
    gyro = [fbk.gyroX; fbk.gyroY; fbk.gyroZ];
    %norm(accel)
    
%     if toc > 5
%         poseFilter1.resetYaw();
%         tic;
%     end
    
    poseFilter1.update( accel(:,1), gyro(:,1), filterTime(1) );
%    poseFilter2.update( accel(:,1), gyro(:,1), filterTime(1) );
    T(:,:,1) = poseFilter1.getPose();
%    T(:,:,2) = poseFilter2.getPose();
%     poseFilter2.update( accel, gyro, filterTime );
%     T(:,:,2) = poseFilter2.getPose();
%     P = poseFilter2.getCovariance();
%     state = poseFilter2.getState();
%     state.xyzAccel
    
%     figure(102);
%     imagesc(P);
%     drawnow;

%     RPY = SpinCalc('DCMtoEA313',T(1:3,1:3,:),1E-6,0);
%     rollAngles = RPY(:,1) - 180;
%     rollAngles(2) = -rollAngles(2);
%     rollAngles;
% 
%     leanAngle = mean(rollAngles);
    
    animStruct = drawAxes( animStruct, T, axisLen );
    drawnow;
    
    if isFirstDraw
        isFirstDraw = false;
        axis equal;
        grid on;
        view(3);
        plotLims = [-.11 .11];
        xlim(plotLims);
        ylim(plotLims);
        zlim(plotLims);  
    end
    
%     cmd.velocity = -fbk.gyroZ;
%     group.set(cmd);

end


%%
log = group.stopLogFull

