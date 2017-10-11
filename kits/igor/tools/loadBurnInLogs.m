function [ hebiLogs ] = loadBurnInLogs( moduleSN )
%LOADBREAKINLOGS Summary of this function goes here
%   Load a bunch of logs and return a cell array of log structs.
%
% Dave Rollinson
% Feb 2017

    logFilePath = ['\\HEBINAS\Logs\X_Logs\BurnIn\' moduleSN '\'];

    fileInfo = dir( [logFilePath '*.hebilog'] );

    logFiles = {fileInfo.name};
    numLogs = length(logFiles);

    for i=1:numLogs
        disp([ 'Loading ' num2str(i) ' of ' num2str(numLogs) ': ' ...
               logFiles{i} ' - ' num2str(fileInfo(i).bytes/1E6,'%0.1f') ' MB']);

    %     hebiLogs{i} = ...
    %             struct(HebiUtils.convertGroupLog(logFiles{i},'view','full'));
        hebiLogs{i} =  struct( HebiUtils.convertGroupLog( ...
                                [logFilePath logFiles{i}] ) );    
    end

    %%
    for i=1:length(logFiles) 
        
        if i==1
            clf(figure(101));
            clf(figure(102));
            torqueOffset = mean(hebiLogs{i}.torque);
        end
        pause;
        
        figure(101);
        subplot(2,1,1);
        plot( hebiLogs{i}.time, hebiLogs{i}.motorCurrent );
        hold on;
        title([moduleSN ': Motor Current - Log ' num2str(i) ' of ' num2str(numLogs)]);
        xlabel('time (sec)');
        ylabel('current (A)');

        subplot(2,1,2);
        plot( hebiLogs{i}.time, hebiLogs{i}.windingTemperature );
        hold on;
        title([moduleSN ': Winding Temperature - Log ' num2str(i) ' of ' num2str(numLogs)]);
        xlabel('time (sec)');
        ylabel('temp (A)');

        figure(102);
        plot( hebiLogs{i}.time, hebiLogs{i}.torque - torqueOffset);
        hold on;
        title([moduleSN ': Torque - Log ' num2str(i) ' of ' num2str(numLogs)]);
        xlabel('time (sec)');
        ylabel('torque (N-m )');
    end

end

