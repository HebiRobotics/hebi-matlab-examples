% Test IO Board for MSI Joystick
%
% Dave Rollinson
% Mar 2017

joyMSI = HebiLookup.newGroupFromNames('*',{'MSI_IO_BOARD'});
joyMSI.startLog();

logDuration = 10;  % sec
logTimer = tic;
while toc(logTimer) < logDuration
    
    getMSIJoyCommands(joyMSI);
    pause(.1);
    
end

log = joyMSI.stopLogIO();