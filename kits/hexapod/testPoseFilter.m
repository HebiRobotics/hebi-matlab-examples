% Testing out the pose filter API
%
% Dave Rollinson
% Mar 2017

clear *;
close all;

group = HebiLookup.newGroupFromNames('*',{'X-80032'});
group.setFeedbackFrequency(100);
fbk = group.getNextFeedbackFull();

poseFilter = HebiPoseFilter();
% poseFilter.setGyroScale( 1 );
poseFilter.setMaxAccelWeight( 0.2 );
% poseFilter.setAccelNormBias( 9.81 );
poseFilter.setMaxAccelNormDev( .5 );

% Start the filter with 'clean' data to ensure it initializes well
accel = [0.01; 0.01; 9.81];
gyro = [0.01; 0.01; 0.01];
poseFilter.update( accel, gyro, fbk.hwTxTime );

animStruct.fig = figure(101);
axisLen = .1*ones(3,1);
isFirstDraw = true;

filterTime = 0;

while true
    fbk = group.getNextFeedbackFull();
    dt = fbk.hwTxTime - filterTime;
    filterTime = fbk.hwTxTime;
    
    accel = [fbk.accelX; fbk.accelY; fbk.accelZ];
    gyro = [fbk.gyroX; fbk.gyroY; fbk.gyroZ];
    
    poseFilter.update( accel, gyro, filterTime );
    T = poseFilter.getPose();
    
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

    T(3,1:3)

end