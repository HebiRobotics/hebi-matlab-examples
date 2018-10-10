%%
% Send commands to the arm to keep it from timing out
fbk = armGroup.getNextFeedback();
cmd.position = fbk.positionCmd;
cmd.velocity = fbk.velocityCmd;
cmd.effort = fbk.effortCmd;
armGroup.send(cmd);

% Reset encoders (old way)!
cmdIO.(setX) = 0;
cmdIO.(setY) = 0;
ioGroup.send(cmdIO);
armGroup.send(cmd);
pause(0.2);

% % Reset encoders (new way)
% cmdIO.(resetXY) = 1; 
% ioGroup.send(cmdIO);
% pause(0.01);
% 
% cmdIO.(resetXY) = 0; 
% ioGroup.send(cmdIO);
% pause(0.01);

cmdIO.(scannerSetX) = 1; % DIN 1 (preset encoder 1)
cmdIO.(scannerSetY) = 1; % DIN 2 (preset encoder 2)
ioGroup.send(cmdIO);
armGroup.send(cmd);
pause(0.1);

cmdIO.(scannerSetX) = 0; % DIN 1 (preset encoder 1)
cmdIO.(scannerSetY) = 0; % DIN 2 (preset encoder 2)
ioGroup.send(cmdIO);
armGroup.send(cmd);
pause(0.1);

%%
% Clear data
cmdIO.(scannerClearData) = 1; % Digital Input 3 (Clear data)
ioGroup.send(cmdIO);
armGroup.send(cmd);
pause(0.1);

cmdIO.(scannerClearData) = 0; % Digital Input 3 (Clear data)
ioGroup.send(cmdIO);
armGroup.send(cmd);
pause(0.1);