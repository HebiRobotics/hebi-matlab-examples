%%
% Reset encoders!
cmdIO.e1 = 0; %encoder 1
cmdIO.e3 = 0; %encoder 2
ioGroup.send(cmdIO);
pause(0.5);

cmdIO.e5 = 1; % Digital input 1 (preset encoder 1)
cmdIO.e6 = 1; % Digital input 2 (preset encoder 2)
ioGroup.send(cmdIO);
pause(0.1);

cmdIO.e5 = 0; % Digital input 1 (preset encoder 1)
cmdIO.e6 = 0; % Digital input 2 (preset encoder 2)
ioGroup.send(cmdIO);
pause(0.1);

%%
%Clear data
cmdIO.e7 = 1; % Digital Input 3 (Clear data)
ioGroup.send(cmdIO);
pause(0.1);

cmdIO.e7 = 0; % Digital Input 3 (Clear data)
ioGroup.send(cmdIO);
pause(0.1);