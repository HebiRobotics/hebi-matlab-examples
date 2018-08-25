function [pipeCenters, pipeSurfacePoints, elbowSweepAngles] = ...
       getPipeModelPoints( bendRadius, pipeDiameter, elbowSweepAngle, ...
                           elbowOriginXYZ, elbowSweepRes, pipeSurfaceRes )
% GETPIPEMODELPOINTS takes in parameters of an ideal pipe elbow and
% generates a bunch of points along the surface of the elbow.
            
    if size(elbowOriginXYZ,2) > 1
        elbowOriginXYZ = elbowOriginXYZ';
    end

    pipeRadius = pipeDiameter / 2;
    
    if nargin < 5
        elbowSweepRes = 90; % number of points along elbow sweep
        pipeSurfaceRes = 90; % number of points around circumference
    end

    elbowSweepAngles = linspace( elbowSweepAngle, 0, elbowSweepRes );
    p = linspace( 0, 2*pi, pipeSurfaceRes );
    
    pipeCenters = nan(3,elbowSweepRes);
    pipeSurfacePoints = nan(3,pipeSurfaceRes,elbowSweepRes);
    
    for i=1:length(elbowSweepAngles)

        pipeCenters(:,i) = [ bendRadius*cos(elbowSweepAngles(i));
                             0;
                             bendRadius*sin(elbowSweepAngles(i)); ];

        baseCircle = [ pipeRadius * -cos(p);
                       pipeRadius * sin(p);
                       zeros(size(p)) ];

        pipeCircle = R_y(-elbowSweepAngles(i)) * baseCircle;

        pipeSurfacePoints(:,:,i) = pipeCircle + pipeCenters(:,i);

    end
    
    % Shift everything to the origin
    pipeCenters = pipeCenters + elbowOriginXYZ;
    pipeSurfacePoints = pipeSurfacePoints + elbowOriginXYZ;
    
end


