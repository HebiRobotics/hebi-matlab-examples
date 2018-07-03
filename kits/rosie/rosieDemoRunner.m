function rosieDemoRunner()

    % Close any previously opened figures
    close all;

    % Make sure libraries and paths are loaded appropriately
    localDir = fileparts(mfilename('fullpath'));
    run(fullfile(localDir, 'startup.m'));
    
    mobileBaseType = 'omni';
    
    switch lower(mobileBaseType)
        case 'diff-drive'
            demoFunction = @rosieDiffDriveDemo;
        case 'omni'
            demoFunction = @rosieOmniDriveDemo;
        case 'mecanum'
            % demoFunction = @rosieMecanumDriveDemo;
            disp('Not Yet Implemented');
        otherwise
            disp('Did not recognize mobile base type');
            return;
    end
                   
    % Start the demo
    while true
        try
            demoFunction();
        catch me
            disp(me.message);
        end
    end
    
end
