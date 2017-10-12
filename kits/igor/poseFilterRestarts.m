% Testing out the pose filter API
%
% Dave Rollinson
% Mar 2017

clear *;
close all;

% group = HebiLookup.newGroupFromNames('*',{'X-00141'});
% fbkRate = group.setFeedbackFrequency(200);
% pause(.1);
% fbk = group.getNextFeedbackFull();
% timeStart = fbk.hwTxTime;

nanCount = 0;
totalCount = 0;

accelBad = [];
accelGood = [];

% Keep initializing filter until it works
while true
    
%     fbk = group.getNextFeedbackFull();

%     % USE DATA FROM A MODULE    
%     accel = [fbk.accelX; fbk.accelY; fbk.accelZ];  % fails ~30/100 
%                                                    % depends on
%                                                    % orientaiton
%     gyro = [fbk.gyroX; fbk.gyroY; fbk.gyroZ];
%     filterTime = fbk.hwTxTime - timeStart;
  
    % accel = [fbk.accelX; fbk.accelY; fbk.accelZ];
    accel = [ .001*(rand-.5) .001*(rand-.5) 10*rand ];  % fails ~2/100
                                                      % Seems to fail when
                                                      % close to 9.81 on Z
    %accel = [ .1 .1 9.81 ];  % Always fails
    %accel = [ -.1 -.1 9.81 ];  % Never fails
    %accel = [ .1 -.1 9.81 ];  % Always fails
    %accel = [ -.1 .1 9.81 ];  % Never fails
    %accel = [ 9.81 .1 -.1 ];  % Always fails
    %accel = [ 9.81 .1 .1 ];  % Never fails
    
    gyro = [0.01; 0.01; 0.01];
    filterTime = 0.0;
    
    
    
    poseFilter = HebiPoseFilter();
    poseFilter.setGyroScale( 1 );
    poseFilter.setMaxAccelWeight( .01 );
    poseFilter.setMaxAccelNormDev( .3 );
    
    %poseFilter.update( [0.01 0.01 9.8], [0.010 0.01 0.01], filterTime );
    poseFilter.update( accel, gyro, filterTime );
    
    T_init(:,:) = poseFilter.getPose();
    
    if any(isnan(T_init(:)))
        nanCount = nanCount+1;
        totalCount = totalCount+1;
        accelBad(end+1,:) = accel';
    else
        totalCount = totalCount+1;
        accelGood(end+1,:) = accel';
    end
    
    nanCount / totalCount
end
