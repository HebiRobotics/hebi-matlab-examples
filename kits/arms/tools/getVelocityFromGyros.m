function [ velocity ] = getVelocityFromGyros( frames, fbk)
%getVelocityFromGyros estimates velocity by using gyros
%   
% Estimates joint velocities by comparing two gyros, one before, and one
% after the output. The last module's velocity is set to fbk.velocity as it
% does not have a distal gyro to compare with.
%
%   Example
%    % Estimate velocities based on gyro readings
%    frames = kin.getFK('output', fbk.position);
%    frames = frames(:,:,dofs);
%    gyroVel = getVelocityFromGyros(frames, fbk); 
%
%
%
%   Example
%     % Compare log file. Assumes 'kin' and 'group' are in workspace
%     hebilog = HebiUtils.loadGroupLog('<insert .hebilog name here>');
%     gyroVels = nan(size(hebilog.velocity));
%     
%     for i = 1:length(hebilog.time)
% 
%         % Create dummy feedback struct
%         fbk = struct();
%         fbk.position = hebilog.position(i,:);
%         fbk.velocity = hebilog.velocity(i,:);
%         fbk.gyroX = hebilog.gyroX(i,:);
%         fbk.gyroY = hebilog.gyroY(i,:);
%         fbk.gyroZ = hebilog.gyroZ(i,:);
% 
%         % Calculate velocities
%         frames = kin.getFK('output', fbk.position);
%         frames = frames(:,:,dofs);
%         gyroVels(i,:) = getVelocityFromGyros(frames, fbk);
% 
%     end
%     plot(hebilog.time, hebilog.velocity - gyroVels);
%     legend(group.getInfo().name);

localGyros = [fbk.gyroX; fbk.gyroY; fbk.gyroZ];
velocity = fbk.velocity;

for i = 1:length(velocity)-1

    % transform next gyro reading into current frame
    localFrame = frames(1:3,1:3,i); % pre output
    nextFrame = frames(1:3,1:3,i+1); % post output
    nextGyro = localFrame \ nextFrame * localGyros(:,i+1);
    
    % difference in z axis (aligned with output) is velocity
    velocity(i) = nextGyro(3) - localGyros(3,i);
    
end

