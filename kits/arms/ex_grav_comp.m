%% Setup
% Robot specific setup. Edit as needed.
[group, kin, gravityVec] = setupArm();

% Select whether a 'Admittance' controller should be enabled. It mitigates
% drift when the robot is not being held in place.
enableAdmittanceControl = false;

% Select the duration in seconds
duration = 30;

%% Gravity compensated mode
cmd = CommandStruct();
t0 = tic();
while toc(t0) < duration
    
    % Standard gravity compensation
    fbk = group.getNextFeedback();
    cmd.effort = kin.getGravCompEfforts(fbk.position, gravityVec);
    
    % Mitigate drift when robot is not held in place 
    if enableAdmittanceControl
        error('TODO: implement');
    end
    
    % send commands to robot
    group.set(cmd);
    
end
