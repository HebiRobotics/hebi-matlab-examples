 function [ hebiLogs ] = loadHebiLogs( logView )
%LOADHEBILOGS Brings up a file dialog to choose one or more logs to load.
%
% [ hebiLogs ] = loadHebiLogs( logView )
%
% HEBILOGS is the log file that is loaded, if only one file is chosen.  If
% multiple log files are selected then HEBILOGS will be a cell array of
% logs.  In either case, HEBILOGS can be passed to PLOTHEBILOGS for
% plotting.
%
% LOGVIEW (optional) is a string for which view of a log's feedback should 
% be loaded.  Valid inputs are 'simple','full', or 'debug'.  If no view is
% specified the default is 'simple'.
%
% Example:
%   hebiLogs = loadHebiLogs( 'full' ); % Brings up a file dialog box and
%                                      % will load the full feedback view 
%                                      % of any log files you select.
%
% See also PLOTHEBILOGS, CROPHEBILOGS, DOWNSAMPLEHEBILOGS.
%
% Dave Rollinson
% Nov 2017

    if nargin==0 || isempty(logView)
        logView = 'simple';
    end

    [fileName,pathName] = uigetfile( '*.hebilog', ...
                              '.hebilog Files (*.hebilog)', ...
                              'MultiSelect', 'on' );

    if ~iscell(fileName) && ~ischar(fileName)
        disp('No files chosen.');
        hebiLogs = [];
        return;
    elseif ~iscell( fileName )
        fileNames{1} = fileName;
    else
        fileNames = fileName;
    end

    numLogs = length(fileNames);

    for i=1:numLogs
        fullFileName = [pathName fileNames{i}];
        fileInfo = dir(fullFileName);

        disp([ 'Loading ' num2str(i) ' of ' ...
           num2str(numLogs) ': '  fileNames{i} ' - ' ...
           num2str(fileInfo.bytes/1E6,'%0.1f') ' MB']);

        hebiLogs{i} = HebiUtils.convertGroupLog( ...
            [pathName fileNames{i}], 'view', logView );
    end
    
    if numLogs==1
        hebiLogs = hebiLogs{1};
    end

end

