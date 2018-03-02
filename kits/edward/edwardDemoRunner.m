%%
%run_edward
function edwardDemoRunner()

    % Close any previously opened figures
    close all;
    
    % Note: 'clear *' was removed as functions have a separate 
    % workspace and it wouldn't do anything.

    % Make sure libraries and paths are loaded appropriately
    localDir = fileparts(mfilename('fullpath'));
    run(fullfile(localDir, 'startup.m'));
    
    % Start the demo
    edwardDemo();
    
end
