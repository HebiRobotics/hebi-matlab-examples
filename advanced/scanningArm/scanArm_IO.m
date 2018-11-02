%%
% Reset encoders!
IO_cmd.e1 = 0; %encoder 1
IO_cmd.e3 = 0; %encoder 2
group.send(IO_cmd)
pause(0.5)
IO_cmd.e5 = 1; %Digital input 1 (preset encoder 1)
IO_cmd.e6 = 1; %Digital input 2 (preset encoder 2)
group.send(IO_cmd)
pause(0.1)
IO_cmd.e5 = 0; %Digital input 1 (preset encoder 1)
IO_cmd.e6 = 0; %Digital input 2 (preset encoder 2)
group.send(IO_cmd)

%%
%Clear data
IO_cmd.e8 = 1; %Digital input 3 (Clear data)
group.send(IO_cmd)

pause(0.1)

IO_cmd.e8 = 0; %Digital input 3 (Clear data)
group.send(IO_cmd)

pause(0.1)