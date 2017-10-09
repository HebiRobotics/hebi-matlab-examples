% Testing SpinCalc and Euler Angles
%
% Dave Rollinson
% July 2017

R = eye(3);

tic;

while toc<5
    RPY = SpinCalc('DCMtoEA123',R_z(rand),1E-9,0);
end

