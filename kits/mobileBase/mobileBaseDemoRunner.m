function mobileBaseDemoRunner( chassisType )

    % Close any previously opened figures
    close all;

    % Make sure libraries and paths are loaded appropriately
    localDir = fileparts(mfilename('fullpath'));
    run(fullfile(localDir, 'startup.m'));

    % Start the demo
    while true
        try
            mobileBaseDemo( chassisType );
        catch me
            disp(me.message);
        end
    end
    
end
