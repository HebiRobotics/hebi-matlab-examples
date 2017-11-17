function [ ] = plotHebiLogs( inputLogs, feedbackField, modules, figNum )
%PLOTHEBILOGS Nicely formatted plotting of loaded .hebilog files.
%  
% plotHebiLogs( INPUTLOGS, FEEDBACKFIELD, MODULES, FIGNUM )
%
% INPUTLOGS is a log object or or a cell array of log objects.  Logs can
% come from logging modules online with HebiGroup.stopLog(), or loaded 
% from a saved file using HebiUtils.convertGroupLog().
%
% If INPUTLOGS is empty, then a file dialog will pop up where you can
% select one or more log files to load and plot.
%
% FEEDBACKFIELD is a string corresponding to a feedback field in the log
% object.  If the field is 'position', 'velocity', or 'effort', the plots
% will show both the feedback and commanded values in an upper subplot, and
% it will show the error of the tracked commands in a lower subplot.
%
% MODULES (optional) is an array of module indices you would like to plot.
% If you do not provide this parameter or you pass in [], then it will plot
% all the modules in the group.
%
% FIGNUM (optional) is the figure number you would like use for plots.  If
% multiple log files are plotted then the subsequent figure numbers 
% increment by 1 starting at FIGNUM.  If FIGNUM is not specified, then a 
% new figure will automatically be generated to avoid over-writing figures.
%
% NOTE:
% There is some code in here to conviently plot MSI logs.  In particular,
% if you pass in 'elbow', 'shoulder', or 'base' for the MODULES parameter
% it will plot those modules from the MSI Hexapod.  The plan is to have
% this parameter work on module names from groupInfo once we have that
% loaded from .hebilog files.
%
% Examples:
%    plotHebiLogs( [], 'position' );  % Lets you choose logs, and plots the
%                                     % the commanded and feedback position
%                                     % of all the modules in those logs.
%
%    plotHebiLogs( [], 'position', 1, 100 );  % Same as above, but it only
%                                             % plots the first module in 
%                                             % the group, and the figure 
%                                             % numbers for multiple logs 
%                                             % start at 100.
%
%    plotHebiLogs( myLog, 'voltage' ); % Plots the voltage readings for
%                                      % all of the modules in myLog, which
%                                      % was loaded ahead of time.
%
% See also LOADHEBILOGS, CROPHEBILOGS, DOWNSAMPLEHEBILOGS.
%
% Dave Rollinson
% Nov 2017
    
    % MSI Hexapod Names
    % (placeholder until we can somehow get groupInfo from log)
    legNames = { 'base1', 'shoulder1', 'elbow1', ...
                 'base2', 'shoulder2', 'elbow2', ...
                 'base3', 'shoulder3', 'elbow3', ...
                 'base4', 'shoulder4', 'elbow4', ...
                 'base5', 'shoulder5', 'elbow5', ...
                 'base6', 'shoulder6', 'elbow6' };          
            
    % Open up a dialog to get files if we pass in empty
    if nargin == 0 || isempty(inputLogs)

        inputLogs = loadHebiLogs('full');
        
        if isempty(inputLogs)
            disp('Nothing plotted.');
            return;
        end
    end

    if ~iscell( inputLogs )
        hebiLogs{1} = inputLogs;
    else
        hebiLogs = inputLogs;
    end
 
    numLogs = length(hebiLogs);
    
    % Iterate thru each log and plot the desired feedback
    for i=1:length(hebiLogs) 
        
        % This is currently hard-coded for MSI modules.  In the future it 
        % would be good to somehow get group information that corresponds 
        % to the log and search for string matches to screen the data.
        if nargin > 2 
            if ischar(modules)
                if contains(modules,'base')
                    plotMask = 1:3:18;
                elseif contains(modules,'shoulder')
                    plotMask = 2:3:18;
                elseif contains(modules,'elbow')
                    plotMask = 3:3:18;
                else
                    error('Please specify: base, shoulder or elbow.');
                end
            elseif ~isempty(modules) 
                plotMask = modules;
            else
                plotMask = 1:size(hebiLogs{i}.position,2);
            end
        else
            plotMask = 1:size(hebiLogs{i}.position,2);
        end

        if nargin > 1 && ~isempty(feedbackField)
            
            % Auto-assign figs, or count up user-defined number
            if nargin < 4
                figure;
            else
                figure(figNum + i-1);
            end
            
            % If pos/vel/effort, make a 2x1 subplot
            if strcmp(feedbackField,'position') || ...
               strcmp(feedbackField,'velocity') || ...
               strcmp(feedbackField,'effort')
                ax = subplot(2,1,1);
            else
                ax = axes;
            end
            
            % Clears old data if reusing a figure window
            if i==1
                hold off;
            end
            
            plot( hebiLogs{i}.time, hebiLogs{i}.(feedbackField)(:,plotMask) );
            
            xlabel('time (sec)');
            ylabel([feedbackField ' (' feedbackUnits(feedbackField) ')']);
            title( [feedbackField ' - Log ' ...
                    num2str(i) ' of ' num2str(numLogs)] );
            if nargin > 2  && ischar(modules)
                legend(legNames(plotMask));
            else
                legend(strsplit(num2str(plotMask)));
            end
            xlim([0 hebiLogs{i}.time(end)]);
            
            % If pos/vel/effort, plot commands and error as well
            if strcmp(feedbackField,'position') || ...
                strcmp(feedbackField,'velocity') || ...
                strcmp(feedbackField,'effort')
                
                hold on;
                ax.ColorOrderIndex = 1;
                plot( hebiLogs{i}.time, ...
                   hebiLogs{i}.([feedbackField 'Cmd'])(:,plotMask), '--' );
                hold off;
                
                subplot(2,1,2);

                plot( hebiLogs{i}.time, ...
                      hebiLogs{i}.(feedbackField)(:,plotMask) - ... 
                      hebiLogs{i}.([feedbackField 'Cmd'])(:,plotMask) );

                xlabel('time (sec)');
                ylabel(['errror (' feedbackUnits(feedbackField) ')']);
                title( [feedbackField ' error'] );
                xlim([0 hebiLogs{i}.time(end)]);
            end
            
            
        else
            figure(101);
            subplot(2,1,1);
            if i==1
                hold off;
            end
            plot( hebiLogs{i}.time, hebiLogs{i}.motorCurrent(:,plotMask) );
            hold on;
            title( ['Motor Current - Log ' ...
                    num2str(i) ' of ' num2str(numLogs)] );
            xlabel('time (sec)');
            ylabel('current (A)');
            ylim([0 3.0]);
            xlim([0 hebiLogs{i}.time(end)]);

            subplot(2,1,2);
            if i==1
                hold off;
            end
            plot( hebiLogs{i}.time, hebiLogs{i}.windingTemperature(:,plotMask) );
            hold on;
            title( ['Winding Temperature - Log ' ...
                   num2str(i) ' of ' num2str(numLogs)] );
            xlabel('time (sec)');
            ylabel('temp (C)');
            ylim([0 175]);
            xlim([0 hebiLogs{i}.time(end)]);

            figure(102);
            if i==1
                hold off;
            end
            plot( hebiLogs{i}.time, hebiLogs{i}.effort(:,plotMask));
            hold on;
            title( ['Torque - Log ' num2str(i) ...
                    ' of ' num2str(numLogs)] );
            xlabel('time (sec)');
            ylabel('effort (N-m )');
            xlim([0 hebiLogs{i}.time(end)]);
        end
    end

end

