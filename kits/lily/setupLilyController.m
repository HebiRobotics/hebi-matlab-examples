function [ controllerGroup ] = setupLilyController( robotName, controllerName )
%SETUPCONTROLLER Set up a group that contains a single mobile device for
%controlling the Lily hexapod robot.

    % Group for the iPad / Mobile Device that's used for the controller.
    % This gets done continuously in a loops since it is likely that the
    % app is closed or the device is off the network when starting the
    % script.
    if nargin>1 && ~isempty(controllerName)
        fprintf('Connecting to controller...');
        retryDelayTime = 2.0;
        while true
            try
                controllerGroup = ...
                    HebiLookup.newGroupFromNames( robotName, controllerName );
                fprintf('DONE.\n');
                break;
            catch
                disp(['  Did not find controller: ' robotName '|' controllerName]);
                pause( retryDelayTime );
            end
        end

        % Configure the iPad screen
        cmdIO = IoCommandStruct();
        cmdIO.a3 = 0.00001;  % Set slider 'a3' to snap to center.  Uses a 
                          	 % small value because there's a bug at 0.
        cmdIO.e1 = 1;   % Make button 'b1' highlight, so we know it is used.
        cmdIO.e8 = 1;   % Make button 'b8' highlight, so we know it is used.
        cmdIO.b1 = 1;   % Make button 'b1' a toggle.
        
        ledColor = 'g';
      
        % Set the button configs, using acknowledgements.
        numSends = 0;
        maxSends = 10;
        ack = false;
        fprintf('Setting up to controller...');
        while ~ack
            ack = controllerGroup.send( cmdIO, ...
                                          'led', ledColor, ...
                                          'ack', true );
            numSends = numSends + 1;
            if numSends > maxSends
                % % Commented out due to bug with Mobile I/O + Matlab API
                % disp('Did not receive acknowledgement from controller.');
                break;
            end
        end
        fprintf('DONE.\n');
    else
        groups.controller = [];
    end
end

