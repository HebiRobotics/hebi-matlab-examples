% Testing out equations for generating an 'ideal' pipe bend.
%
% Dave Rollinson
% May 2018

%%
% NUKE THE WORKSPACE
clear *;
close all;

%%
% Parameters
bendRadius= .300;  
pipeDiameter = .220; 
elbowSweepAngle = deg2rad(90);
elbowOrigin = [ 0.200;
                0.00;
               -0.500 ];

[pipeCenters, pipeSurfacePoints, elbowSweepAngles] = getPipeModelPoints( ...
            bendRadius, pipeDiameter, elbowSweepAngle, elbowOrigin );

%% 
% Visualize

for i=1:length(elbowSweepAngles)
    plot3( pipeSurfacePoints(1,:,i), ...
           pipeSurfacePoints(2,:,i), ... 
           pipeSurfacePoints(3,:,i), 'b:' ); 
    hold on;
end


axis equal;
grid;
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
