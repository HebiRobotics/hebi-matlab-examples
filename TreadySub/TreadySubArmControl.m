% -------------------------------------------------------------------------
% NOTE
% Controlling only torques (or forces) will always exhibit some amount of
% drift due to noise in the sensors and a non-perfect model of the robot.
% This can be mitigated by adding an extra controller that can add torques
% to remain at a position when the robot is not actively beind held.
% -------------------------------------------------------------------------

startup();
%% Setup
% Robot specific setup. Edit as needed.
[leader, leaderKin, leaderParams] = setupArm('A-2303-01', 'Tready-Sub-Leader');
[follow, followKin, followParams] = setupArm('A-2303-01', 'Tready-Sub-Follower');


follow.trajGen.setMinDuration(2.0);   % Min move time for 'small' movements
                                      % (default is 1.0)
follow.trajGen.setSpeedFactor(0.5);  % Slow down movements to a safer speed.
                                      % (default is 1.0)

% Setup camera mast
mastGroup = HebiLookup.newGroupFromNames('Tready-Sub', {'J1_pan', 'J2_tilt'});
mastCmd = CommandStruct();
mastFbk = mastGroup.getNextFeedback();
mastCmd.position = mastFbk.position;
% Setup Visualization
framesDisplay = FrameDisplay();

%initialize keyboard
kb = HebiKeyboard();

% Start Logging
leader.group.startLog('dir', '/logs/leader');
follow.group.startLog('dir', '/logs/follow');

homePose = [1.0, -1.35, 0.52, -0.8, 1.35, -1.2, -0.15];
follow.update();
follow.clearGoal(); % in case we re-run this section
follow.setGoal(homePose);
abortFlag = false;
while ~follow.isAtGoal() && ~abortFlag
   follow.update();
   follow.send();

   % Check for keyboard input and break out of the main loop
   % if the ESC key is pressed.
   key = read(kb);
   abortFlag = key.ESC;
end


%%
followcmd = CommandStruct();
leadercmd = CommandStruct();

%gravity compensation warning
disp ('Commanded gravity-compensated zero torques to the arm');
disp ('Press ESC to Stop');
disp ('Press 1 to start follower and 2 to pause follower')
disp ('Press 3 to activate haptic feedback, and 4 to de-activate')
disp ('press 5 to re-home the follower arm')

STATES = ['off', 'aligning', 'aligned', 'homing'];
state = 'off';
hapticFeedback = false;

%Haptic gain controls the sensitivity of the feedback. A higher number will
%make the leader harder to drive but you will feel the feedback from the
%follower more
hapticGain =  10 * [30 15 30 20 10 3 2];
hapticLimit = 100;

while ~key.ESC

    key = read(kb);

    % startup follower arm and enable haptics on user input
    if key.keys('1')
        state = 'aligning';
        follow.clearGoal();
        follow.setGoal(leader.state.fbk.position);
    elseif key.keys('2')
        state = 'off';
        hapticFeedback = false;
        % pause at current pose when stopping tracking
        follow.clearGoal();
        follow.setGoal(follow.state.fbk.positionCmd);
    elseif key.keys('3')
        hapticFeedback = true;
    elseif key.keys('4')
        hapticFeedback = false;
    elseif key.keys('5')
        state = 'homing';
        hapticFeedback = false;
        follow.clearGoal(); % in case we re-run this section
        follow.setGoal(homePose);
    elseif key.LEFT
        mastCmd.position(1) = mastCmd.position(1) + 0.01;
    elseif key.RIGHT
        mastCmd.position(1) = mastCmd.position(1) - 0.01;
    elseif key.UP
        mastCmd.position(2) = mastCmd.position(2) - 0.01;
    elseif key.DOWN
        mastCmd.position(2) = mastCmd.position(2) + 0.01;
    end

    % Get leader sensor data
    try
        leader.update('timeout', 0.01);
        follow.update('timeout', 0.01);
        leaderfbk = leader.plugins{2}.group.getNextFeedback();
        % followfbk = follow.plugins{2}.group.getNextFeedback();
    catch
        leader.send();
        % follow.send();
        continue;
    end

    leaderPos = leader.state.fbk.position;
    leaderEff = leader.state.fbk.effort;
    leaderVel = leader.state.fbk.velocity;
    leaderShoulderPos = leaderfbk.position;

    if strcmp(state, 'aligned')

        % mirror leader position to follower
        follow.state.cmdPos = leaderPos;
        follow.state.cmdVel = leaderVel;
        % follow.state.cmdEffort = leaderEff;
        followcmd.position = -follow.state.cmdPos(2);
        followcmd.effort = -follow.state.cmdEffort(2);

        if hapticFeedback
            followPos = follow.state.fbk.position;
            followShoulerPos = followfbk.position;

            %calc feedback into leader
            posDiff = (leaderPos - followPos);
            hapticEffort = hapticGain .* posDiff .* abs(posDiff);
            hapticEffort = min(max(hapticEffort, -hapticLimit), hapticLimit);
            leader.state.cmdEffort = leader.state.cmdEffort - hapticEffort;

            %calc double shoulder feedback
            leadercmd.effort = -leader.state.cmdEffort(2);
            % shoulderPosDiff = (leaderShoulderPos - followShoulerPos);
            % hapticShoulderEffort = hapticGain(2) * shoulderPosDiff.^2 .*sign(shoulderPosDiff);
            % leadercmd.effort = leadercmd.effort - hapticShoulderEffort;
        end

        follow.plugins{2}.group.send(followcmd);

    elseif strcmp(state, 'aligning')
        if follow.isAtGoal()
            hapticFeedback = false;
            follow.clearGoal();
            state = 'aligned';
            disp ('Follower aligned with Leader')
        end
    elseif strcmp(state, 'homing')
        if follow.isAtGoal()
            disp ('Follower Arm in Home position');
            state = 'off';
        end
    end

    mastGroup.send(mastCmd);

    leader.send();
    follow.send();


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
