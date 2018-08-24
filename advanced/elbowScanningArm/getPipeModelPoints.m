function [pipeCenters, pipeSurfacePoints, elbowSweepAngles] = ...
       getPipeModelPoints( bendRadius, pipeDiameter, elbowOriginXYZ, ...
                           elbowSweepRes, pipeSurfaceRes )
            
    if size(elbowOriginXYZ,2) > 1
        elbowOriginXYZ = elbowOriginXYZ';
    end

    pipeRadius = pipeDiameter / 2;
    
    elbowSweepAngle = pi/2;
    
    if nargin < 4
        elbowSweepRes = 90; % num points around elbow sweep
        pipeSurfaceRes = 90; % num points around circumference
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


