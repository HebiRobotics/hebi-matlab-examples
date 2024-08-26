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
pause(1);

% Instantiate the arm kit based on the config files in config/${name}.yaml
% If your kit has a gas spring, you need to uncomment the offset lines
% in the corresponding config file.
[ arm, params ] = setupArm( 'leader' );
[ arm2, params2 ] = setupArm( 'follower' );

arm2.trajGen.setMinDuration(0.2);


localDir = params.localDir;
enableLogging = true;

% Start background logging 
if enableLogging
   logFile = arm.group.startLog('dir',[localDir '/logs']); 
end

%% Gravity compensated mode
disp('Commanding gravity-compensated torques to both arms.');
disp('Commanding leader arm to follower arm.');
disp('Commanding follower arm torque errors to leader arm.');
disp('Press ESC to stop.');

% Keyboard input
kb = HebiKeyboard();
keys = read(kb);
while ~keys.ESC   
    keys = read(kb);
    
    arm.update();
    arm2.update();
    
    arm2.setGoal( arm.state.fbk.position );
    
    effortError = arm2.state.fbk.effortCmd - arm2.state.fbk.effort;
    
    % Tweak to reduce the applied torque errors when they're small.  This
    % helps make the arm feel better when there are only small errors due
    % to modeling / spring error, but still apply torques when there's
    % significant error from collisions / disturbances
    effortScales = 0.5 * [16 16 16 9 9 3];  % Scales depending on joint
    fudgeFactor = abs(effortError) ./ effortScales;
    fudgeFactor = min( fudgeFactor, 1.0 ); % Cap at 1 (don't amplify errors)
    
    modifiedError = fudgeFactor .* effortError;
    
    arm.state.cmdEffort = arm.state.cmdEffort + modifiedError;
    
    arm.send();
    arm2.send();
end

%%
if enableLogging
    
   hebilog = arm.group.stopLogFull();
   
   % Plot tracking / error from the joints in the arm.  Note that there
   % will not by any 'error' in tracking for position and velocity, since
   % this example only commands effort.
   HebiUtils.plotLogs(hebilog, 'position');
   HebiUtils.plotLogs(hebilog, 'velocity');
   HebiUtils.plotLogs(hebilog, 'effort');
   
   % Plot the end-effectory trajectory and error
   kinematics_analysis( hebilog, arm.kin );
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   % Feel free to put more plotting code here %
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
