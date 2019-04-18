function [ cmd ] = makeLegCmds( cmd, legIndex, ...
                                legCmdAngles, legCmdAngVels, legCmdEfforts )
%MAKELEGCMDS Fill in the commands for Florence, handling the doubled DoFs
%for the the knee and hip.

        cmdPosition = nan(1,16);
        cmdVelocity = nan(1,16);
        cmdEffort = nan(1,16);

    for leg=1:2
        % Assume input commands are row vectors
        cmdPosition(legIndex{leg}) = legCmdAngles(leg,:);
        cmdPosition(legIndex{leg}(3)+1) = -cmdPosition(legIndex{leg}(3));
        cmdPosition(legIndex{leg}(4)+1) = -cmdPosition(legIndex{leg}(4));

        % Assume velocity commands are column vectors, since they come from
        % a matrix math operation.
        cmdVelocity(legIndex{leg}) = legCmdAngVels(:,leg);
        cmdVelocity(legIndex{leg}(3)+1) = -cmdVelocity(legIndex{leg}(3));
        cmdVelocity(legIndex{leg}(4)+1) = -cmdVelocity(legIndex{leg}(4));

        % Assume effort commands are column vectors, since they come from
        % a matrix math operation.
        cmdEffort(legIndex{leg}) = legCmdEfforts(:,leg);
        cmdEffort(legIndex{leg}(3)) = cmdEffort(legIndex{leg}(3))/2;
        cmdEffort(legIndex{leg}(4)) = cmdEffort(legIndex{leg}(4))/2;
        cmdEffort(legIndex{leg}(3)+1) = -cmdEffort(legIndex{leg}(3));
        cmdEffort(legIndex{leg}(4)+1) = -cmdEffort(legIndex{leg}(4));
    end
    
    cmd.position = cmdPosition;
    cmd.velocity = cmdVelocity;
    cmd.effort = cmdEffort;

end

