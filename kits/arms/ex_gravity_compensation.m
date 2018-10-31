% Arm Gravity Compensation Demo
%
% Features:      Demo where the arm can be interacted with and moved around
%                while in a zero-force gravity-compensated mode.
%
% Requirements:  MATLAB 2013b or higher
%
% Author:        Dave Rollinson
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
% Date:          Oct 2018

% Copyright 2017-2018 HEBI Robotics

%% Setup
clear *;
close all;

HebiLookup.initialize();

localDir = fileparts(mfilename('fullpath'));

armName = '6-DoF + gripper';
armFamily = 'Arm';

[ armGroup, armKin, armParams ] = setupArm( armName, armFamily );
gravityVec = armParams.gravityVec;
effortOffset = armParams.effortOffset;

enableLogging = true;

% Start background logging 
if enableLogging
   logFile = armGroup.startLog('dir',[localDir '/logs']); 
end

%% Gravity compensated mode
cmd = CommandStruct();

% Keyboard input
kb = HebiKeyboard();
keys = read(kb);

disp('Commanded gravity-compensated zero torques to the arm.');
disp('Press ESC to stop.');

while ~keys.ESC   
    
    % Gather sensor data from the arm
    fbk = armGroup.getNextFeedback();
    
    % Calculate required torques to negate gravity at current position
    cmd.effort = armKin.getGravCompEfforts( fbk.position, gravityVec ) ...
        + effortOffset;
    
    % Send to robot
    armGroup.send(cmd);

    % Check for new key presses on the keyboard
    keys = read(kb);
    
end

%%
if enableLogging
    
   hebilog = armGroup.stopLogFull();
   
   % Plot tracking / error from the joints in the arm.  Note that there
   % will not by any 'error' in tracking for position and velocity, since
   % this example only commands effort.
   HebiUtils.plotLogs(hebilog, 'position');
   HebiUtils.plotLogs(hebilog, 'velocity');
   HebiUtils.plotLogs(hebilog, 'effort');
   
   % Plot the end-effectory trajectory and error
   kinematics_analysis( hebilog, armKin );
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   % Feel free to put more plotting code here %
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
