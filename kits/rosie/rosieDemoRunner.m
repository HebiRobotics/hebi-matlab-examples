function rosieDemoRunner( chassisType )

    % Close any previously opened figures
    close all;

    % Make sure libraries and paths are loaded appropriately
    localDir = fileparts(mfilename('fullpath'));
    run(fullfile(localDir, 'startup.m'));

    % Start the demo
    while true
        try
            rosieDemo( chassisType );
        catch me
            disp(me.message);
        end
    end
    
end
