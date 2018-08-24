% Rotate a Figure to make animations
%
% Dave Rollinson
% May 2018

clear *;
close all;

% Load the figure
figureName = 'pipeCoverage';
openfig( figureName );

azimuth = -45;
elevation = 20;

view(azimuth,elevation);

for i=1:360
    
    newAzimuth = azimuth + i;
    
    view(newAzimuth,elevation);
    drawnow;
    
    saveFrameToFile( gcf, [figureName '\' figureName], i );
end

