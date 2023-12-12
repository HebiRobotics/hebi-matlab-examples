% -------------------------------------------------------------------------
% NOTE
% Controlling only torques (or forces) will always exhibit some amount of 
% drift due to noise in the sensors and a non-perfect model of the robot. 
% This can be mitigated by adding an extra controller that can add torques
% to remain at a position when the robot is not actively beind held.
% -------------------------------------------------------------------------

%% Setup
% Robot specific setup. Edit as needed.
[leader, leaderKin, leaderParams] = setupArm('A-2303-01', 'Dow-Leader');
[follow, followKin, followParams] = setupArm('A-2303-01', 'Dow-Follower');

% Setup Visualization
framesDisplay = FrameDisplay();

% Start Logging
leader.group.startLog('dir', '/logs/leader');
follow.group.startLog('dir', '/logs/follow');

%% 
cmd = CommandStruct();

%gravity compensation warning
disp ('Commanded gravity-compensated zero torques to the arm');
disp ('Press ESC to Stop');
disp ('Press alt to start follower and space to stop follower')
disp ('Press ctrl to activate haptic feedback, and shift to de-activate')

%initialize keyboard
kb = HebiKeyboard();
keys = read(kb);
includeFollower = false;
hapticFeedback = false;

%Haptic gain controls the sensitivity of the feedback. A higher number will
%make the leader harder to drive but you will feel the feedback from the
%follower more
hapticGain =  10 * [30 30 30 20 10 10 10]; 

while ~keys.ESC
    
    keys = read(kb);
    
    % startup follower arm and enable haptics on user input
    if keys.ALT
        includeFollower = true;
    elseif keys.SPACE
        includeFollower = false;
        hapticFeedback = false;
    elseif keys.CTRL
        hapticFeedback = true;
    elseif keys.SHIFT
        hapticFeedback = false;
    end
    
    
    % Get leader sensor data
    leader.update();    
    leaderFbk = leader.group.getNextFeedback();
    leaderPos = leaderFbk.position;
    leaderEff = leaderFbk.effort;
    leaderVel = leaderFbk.velocity;
       
    
    if includeFollower
        
        % mirror leader position to follower 
        follow.update();
        cmd.position = leaderPos;
        cmd.effort = leaderEff;
        cmd.velocity = leaderVel;
        follow.group.send(cmd);
        
        if hapticFeedback
            
            followFbk = follow.group.getNextFeedback();
            followPos = followFbk.position;            
            
            %calc feedback into leader             
            posDiff = (leaderPos - followPos);
            hapticEffort = hapticGain .* posDiff.^2 .* sign(posDiff);
            leader.state.cmdEffort = leader.state.cmdEffort - hapticEffort;
           
        end
        

    end
    
    leader.send();
    
%     % leader Visualization
%     leaderPositions = leaderFbk.position; 
%     leaderFrames = leader.kin.getFK('output', leaderPositions);
%     framesDisplay.setFrames(leaderFrames)
%     drawnow;
%     axis equal
    
%     follower Visualization
%     followFbk = follow.group.getNextFeedback();
%     followPositions = followFbk.position; 
%     followFrames = follow.kin.getFK('output', followPositions);
%     framesDisplay.setFrames(followFrames)
%     drawnow;
%     axis equal
end

%user feedback
    disp('Stopped')

% Stop Logging
leaderLog = leader.group.stopLogFull();
followLog = follow.group.stopLogFull();