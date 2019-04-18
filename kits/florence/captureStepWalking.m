% Initial implementation of Reference Trajectory Generation
%
% From:
% "Balanced Walking with Capture Steps"
% Missura and Behnke, 2014
%
% Coordinate Convention (robot perspective):
% +X - forward
% +Y - right
% +Z - up

clear *;
close all;

HebiLookup.initialize();

%%%%%%%%%%%%%%%%%% 
% Setup the iPad %
%%%%%%%%%%%%%%%%%%
robotName = 'Florence';
controllerName = '_Controller';
simulate = true;
isFirstDraw = true;

[ groups, kin, robotParams ] = ...
                    setupFlorence( robotName, controllerName, simulate );
controllerGroup = groups.controller;

stepParams = setupCapturePointWalking();

mass = 20;  % kg
h_CoM = 0.5;  % [m]
g = 9.81;        % [m/sec^2]

C = sqrt( g / h_CoM );
lambda = 1;    % [-1,1] support leg indicator

%%%%%%%%%%%%%%%%%%%
% STEP PARAMETERS %
%%%%%%%%%%%%%%%%%%%
alpha = stepParams.alpha;  % lateral distance from foot at step ap ex
sigma = stepParams.sigma;  % max saggital velocity at step apex
delta = stepParams.delta;  % min support exchange location
omega = stepParams.omega;  % max support exchange location

Zx_lim = stepParams.halfFootLength;
Zy_lim = stepParams.halfFootWidth;

minStepTime = stepParams.minStepTime;  % used if we need to step 'immediately'.
T_min = minStepTime;

ankleGain = 0.75;  % Ratio of full ZMP to apply as ankle torque. Setting to 
                  % less than 1 make the robot more 'stumbly', and
                  % probably more realistic :-)

%%%%%%%%%%%%%%%%%%%%%%
% INITIAL CONDITIONS %
%%%%%%%%%%%%%%%%%%%%%%
T = 2.0;

V_x = 0;
V_y = 0;
V_theta = 0;

Z_x = 0;
Z_y = 0;

E_y = 0;

F_stance = [0,0];
F_stanceLast = F_stance;

xi = delta + abs(V_y) * (omega - delta);

damping = -0.01; % stability parameter for simulation
c = [  0.00;    % c_x [m]      (CoM x position) 
       0.00;    % cc_x [m/sec] (CoM x velocity)
       0.00;    % ccc_x [m/sec] (CoM x acceleration)
       0.06;    % c_y [m]      (CoM y position)
       0.00;    % cc_y [m/sec] (CoM y velocity)
       0.00 ];  % ccc_y [m/sec] (CoM y acceleration)

cPrime = c;    % Acheivable end-of-step state


% Variable Histories
t_hist = [];
T_hist = [];
T_bar_hist = [];
E_y_hist = [];
Z_hist = nan(0,2);
F_hist = nan(0,2);
F_stance_hist = nan(0,2);
F_swing_hist = nan(0,2);
c_hist = nan(6,0);
cPrime_hist = nan(6,0);   
disturbForce_hist = nan(0,2);

% Get the initial feedback objects that we'll reuse later for the
% controller group.
fbkControllerIO = controllerGroup.getNextFeedbackIO();
fbkControllerMobile = controllerGroup.getNextFeedbackMobile();
latestControllerIO = fbkControllerIO;
latestControllerMobile = fbkControllerMobile;

runTimer = tic;
stepTimer = tic;
tLast = toc(runTimer);

tScale = 1.0;  % Make smaller if you want to slow down visualization
tUpSample = 200;  % Up-sampling between draws of the visualizer

% Gains for disturbances
forceGainX = 50;
forceGainY = 20;

% Fwd Dynamics for Disturbances
B = zeros(6,2);
B(3,1) = 1 / mass;
B(6,2) = 1 / mass;

% Fwd Dynamics for Ankle Torques
G = zeros(6,2);
G(3,1) = -1 / ( mass * h_CoM^2 );  % This negative sign bothers me :-(
G(6,2) = 1 / ( mass * h_CoM^2 );

% Controls
disp(' ');
disp('  ===================================');
disp('  CONTROLS:');
disp('  A7 - Lateral velocity (Y-Axis)');
disp('  A8 - Saggital velocity (X-Axis)');
disp('  A1 - Lateral disturbance (Y-Axis)');
disp('  A2 - Saggital disturbance (X-Axis)');
disp('  B1 - Quit');
disp('  ===================================');
disp('  ');

while true
    
    % Timekeeping
    t = tScale*toc(runTimer);
    dt = t - tLast;
    tLast = t;
    stepTime = toc(stepTimer);
    
    % Read Joystick
    tempFbk = controllerGroup.getNextFeedback( ...
                    fbkControllerIO, fbkControllerMobile, 'timeout', 0 );
    if ~isempty(tempFbk)
        latestontrollerMobile = fbkControllerMobile;
        latestControllerIO = fbkControllerIO;
    end
    
    % Adjust CoM height
    h_CoM = h_CoM + 0.01*latestControllerIO.a3;
    C = sqrt( g / h_CoM );
    
    % Switch legs if its time to 
    if T <= 0 && stepTime > T_min
        
        F_stanceLast = F_stance;
                    
        % Update stance position
        F_stance(1) = F_stance(1) + c(1) + F(1);
        F_stance(2) = F_stance(2) + c(4) + F(2);
        
        % Switch legs
        lambda = -lambda;
        c(1) = -F(1);  % c_x
        c(4) = -F(2);  % c_y

        % Reset nominal step time
        T_bar = 2*tau;
        stepTimer = tic;
        T_min = minStepTime;
    end
    
    % Up-sample the simulation because dt was limited by plotting.
    dt_sim = dt / tUpSample;
    
    % Forward integration of pendulum state
    fwdDynamics = [ 1       dt_sim    0.5*dt_sim^2;
                    0          1         dt_sim;
                   C^2      damping         0      ];
    A = zeros(6,6);
    A(1:3,1:3) = fwdDynamics;
    A(4:6,4:6) = fwdDynamics;    
    
    % Applying external disturbances
    disturbForce = [  latestControllerIO.a2;
                     -latestControllerIO.a1 ];
    disturbForce(1) = forceGainX * disturbForce(1);
    disturbForce(2) = forceGainY * disturbForce(2);

    % Applying ankle torques for ZMP
    ankleTorque = ankleGain * mass * g * [Z_x; Z_y];
    
    for i = 1:tUpSample
        c = A*c + B*disturbForce + G*ankleTorque;
    end
    
    c_x = c(1);
    cc_x = c(2);
    c_y = c(4);
    cc_y = c(5);
    
    c_x_world = F_stance(1) + c_x;
    c_y_world = F_stance(2) + c_y;
    
    V_x = latestControllerIO.a8;      % normalized saggital velocity
    V_y = -latestControllerIO.a7;      % normalized lateral velocity
    V_theta = latestControllerIO.a1;  % normalized rotational velocity
    
    % lateral support exchange location for leading step
    xi = delta + abs(V_y) * (omega - delta);

    % 1/2 step time, time from lateral apex to support exchange location, xi
    tau = 1/C * log(xi/alpha + sqrt( xi^2/alpha^2 - 1 ));
    
    % Ideal step time
    if exist('T_bar','var')
        T_bar = T_bar - dt;
    else
        T_bar = 2*tau;
    end

    % Ideal X and Y direction support exchange states
    s_x = V_x * sigma/C * sinh(C*tau);
    ss_x = V_x * sigma * cosh(C*tau);

    if lambda==sign(V_y)
        s_y = lambda * xi;
    else
        s_y = lambda * delta;
    end
    ss_y = lambda * C * sqrt(s_y^2 - alpha^2);

    % Lateral ZMP Offset
    Z_y = (s_y*2*C*exp(C*T_bar) - c_y*C*(1+exp(2*C*T_bar)) + cc_y*(1-exp(2*C*T_bar))) ...
                / (C*(exp(2*C*T_bar) - 2*exp(C*T_bar) + 1));
            
    % Cap ZMP Offset 
    Z_y = min( Z_y, Zy_lim );
    Z_y = max( Z_y, -Zy_lim );
    
    % Orbital Energy
    E_y = (cc_y^2 - C^2*c_y^2) / 2;
    
               
    % Step Time
    % Using intermediate variables from:
    % "Omnidirectional Capture Steps for Bipedal Walking"
    c1 = (c_y - Z_y) + cc_y/C;
    c2 = (c_y - Z_y) - cc_y/C;
    T = 1/C * log( (s_y-Z_y)/c1 + sqrt( (s_y-Z_y)^2/c1^2 - c2/c1 ) );
    
    % Keep T from going negative, and take only the real part if the sqrt()
    % above goes negative.  This I think prevents some freakouts when in
    % some recovery situations.
    T = real(T);
    T = max(T,-0.01);
    T = min(T,2*tau);
    
    % Hold off on taking a step if we're at risk of falling over
    if (E_y > 0) && (sign(c_y)*sign(cc_y) < 0)
        T = 2*tau;
        T_bar = 2*tau;
    end
    
    % Saggital ZMP Offset
    Z_x = (s_x + ss_x/C - exp(C*T)*(c_x+cc_x/C)) / (1-exp(C*T));
    Z_x = min( Z_x, Zx_lim );
    Z_x = max( Z_x, -Zx_lim );

    % Acheivable end-of-step state
    cPrime_x = (c_x - Z_x)*cosh(C*T) + cc_x/C*sinh(C*T);
    ccPrime_x = (c_x - Z_x)*C*sinh(C*T) + cc_x*cosh(C*T);
    
    % Threshold unstable states
    cPrime_x = min(abs(cPrime_x),1.5*abs(cc_x)) * sign(cPrime_x);
    ccPrime_x = min(abs(ccPrime_x),2.0*abs(cc_x)) * sign(ccPrime_x);
    
    cPrime_y = (c_y - Z_y)*cosh(C*T) + cc_y/C*sinh(C*T);
    ccPrime_y = (c_y - Z_y)*C*sinh(C*T) + cc_y*cosh(C*T);
    
    cPrime(1) = cPrime_x;
    cPrime(2) = ccPrime_x;
    cPrime(4) = cPrime_y;
    cPrime(5) = ccPrime_y;
    
    % Target end-of-step footstep location
    F = [ ccPrime_x/C*tanh(C*tau), lambda*sqrt(ccPrime_y^2/C^2+alpha^2) ];
    F_target = F + [c_x_world c_y_world];
    
    % Simulate a swing foot that gets to the next target
    stepPhase = min( 1 - T/(2*tau), stepTime/minStepTime);
    stepPhase = smoothStep(stepPhase);    
    
    F_swing = (1-stepPhase) * F_stanceLast + ...
                  stepPhase * F_target;      
    
    %%%%%%%%%%%%%%%%%%%%
    % VARIABLE HISTORY %
    %%%%%%%%%%%%%%%%%%%%
    t_hist(end+1) = t;
    T_hist(end+1) = T;
    T_bar_hist(end+1) = T_bar; 
    Z_hist(end+1,:) = [Z_x Z_y];
    F_stance_hist(end+1,:) = F_stance;
    F_swing_hist(end+1,:) = F_swing;
    F_hist(end+1,:) = F;
    c_hist(:,end+1) = c;
    cPrime_hist(:,end+1) = cPrime;
    E_y_hist(end+1) = E_y;
    disturbForce_hist(end+1,:) = disturbForce;
    
    %%%%%%%%%%%%%%
    % VISUALIZE  %
    %%%%%%%%%%%%%%    
    CoM_x = c_x + F_stance(1);
    CoM_y = c_y + F_stance(2);
    CoM_z = sqrt(h_CoM^2-c_y^2-c_x^2);
    plotZ_x = Z_x + F_stance(1);
    plotZ_y = Z_y + F_stance(2);
    
    CoM_x = real(CoM_x);
    CoM_y = real(CoM_y);
    CoM_z = real(CoM_z);
    plotZ_x = real(plotZ_x);
    plotZ_y = real(plotZ_y);
    
    vecPlotScale = 0.01;
    forceVec = [ CoM_x, CoM_x - vecPlotScale*disturbForce(1);
                 CoM_y, CoM_y - vecPlotScale*disturbForce(2);
                 CoM_z, CoM_z ];
    
    
    if isFirstDraw
        visLine(2) = plot3(CoM_x,CoM_y,CoM_z,'k*','linewidth',3,'markersize',10);
        hold on;
        visLine(3) = plot3(plotZ_x,plotZ_y,0,'go','linewidth',3);
        visLine(4) = plot3(F_target(1),F_target(2),0,'bx','linewidth',2,'markersize',10);
        visLine(1) = plot3([F_stance(1) CoM_x],[F_stance(2) CoM_y],[0 CoM_z],'k','linewidth',3);

        visLine(5) = plot3(forceVec(1,:),forceVec(2,:),forceVec(3,:),'r','linewidth',3);
        visLine(6) = plot3([F_swing(1) CoM_x],[F_swing(2) CoM_y],[0 CoM_z],'k');
        
        visAx = visLine(1).Parent;
        visBuffer = 1.0 * h_CoM;

        axis equal;
        grid on;
        xlim([-1.0*h_CoM 1.0*h_CoM]);
        ylim([-1.0*h_CoM 1.0*h_CoM]);
        zlim([-0.1*h_CoM 1.1*h_CoM]);
        title('Walking Controller Visualization');
        xlabel('x (m)');
        ylabel('y (m)');
        zlabel('z (m)');
        % legend('CoM','ZMP','Step Target');
        
        isFirstDraw = false;
        
    else
        set(visLine(1),'xdata',[F_stance(1) CoM_x],'ydata',[F_stance(2) CoM_y],'zdata',[0 CoM_z]);
        set(visLine(2),'xdata',CoM_x,'ydata',CoM_y,'zdata',CoM_z);
        set(visLine(3),'xdata',plotZ_x,'ydata',plotZ_y,'zdata',0);
        set(visLine(4),'xdata',F_target(1),'ydata',F_target(2),'zdata',0);
        set(visLine(5),'xdata',forceVec(1,:),'ydata',forceVec(2,:),'zdata',forceVec(3,:));
        set(visLine(6),'xdata',[F_swing(1) CoM_x],'ydata',[F_swing(2) CoM_y],'zdata',[0 CoM_z]);
        
        set( visAx, 'xlim', [CoM_x-visBuffer CoM_x+visBuffer], ...
                    'ylim', [CoM_y-visBuffer CoM_y+visBuffer] );
    end
    
    drawnow;
    pause(0.01);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % CHECK TO SEE IF WE FELL OVER %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if abs(c(4)) > h_CoM/sqrt(2) || abs(c(1)) > h_CoM/sqrt(2)
        disp('Waahmp waahmp waaaaaaahmp.  Sad trombones.  :-(');
        break;
    end
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    % CHECK FOR QUIT COMMAND %
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    if fbkControllerIO.b1
        disp('Quitting.');
        break;
    end

end

        
% Set the button configs, using acknowledgements.
numSends = 0;
maxSends = 1;  % Increase when ack is fixed
ack = false;

while ~ack
    cmdIO = IoCommandStruct();
    cmdIO.a3 = nan; 
    cmdIO.e1 = 0;  
    cmdIO.b6 = 0;  
    ack = controllerGroup.send( cmdIO, ...
                                'led', [], ...
                                'ack', true );
    numSends = numSends + 1;
    if numSends > maxSends
        % disp('Did not receive acknowledgement from controller.');
        break;
    end
end
        

%%
%%%%%%%%%%%%
% PLOTTING %
%%%%%%%%%%%%

figure(101);

ax = subplot(3,1,1);
plot( t_hist, T_hist );
hold on;
plot( t_hist, T_bar_hist, '--' );
hold off;
title('Step Timing');
xlabel('time (sec)');
ylabel('step time (sec)');
legend('step time','nominal step time');
% axis equal;
xlim([0 t_hist(end)]);
%ylim([-1 1]);
grid on;

ax = subplot(3,1,2);
plot(t_hist,c_hist(1,:),'r');
hold on;
plot(t_hist,c_hist(4,:),'g');
plot(t_hist,cPrime_hist(1,:),'r--');
plot(t_hist,cPrime_hist(4,:),'g--');
plot(t_hist,F_hist(:,1)'+cPrime_hist(1,:),'r:');
plot(t_hist,F_hist(:,2)'+cPrime_hist(4,:),'g:');
% plot(t_hist,Z_hist(:,1));
plot(t_hist,Z_hist(:,2),'k');
hold off;
title('CoM Position - Stance Foot Frame');
xlabel('time (sec)');
ylabel('position (m)');
xlim([0 t_hist(end)]);
ylim([-0.6 0.6]);
legend('c_x','c_y');
grid on;

ax = subplot(3,1,3);
plot(t_hist,c_hist(2,:),'r');
hold on;
plot(t_hist,c_hist(5,:),'g');
hold off;
title('CoM Velocity');
xlabel('time (sec)');
ylabel('velocity (m/sec)');
legend('cc_x','cc_y');
xlim([0 t_hist(end)]);
grid on;


figure(102);
ax = subplot(2,1,1);
plot(t_hist,F_stance_hist(:,1)' + c_hist(1,:),'r');
hold on;
plot(t_hist,F_stance_hist(:,1),'k.');
plot(t_hist,F_swing_hist(:,1),'.','markerSize',1,'color',[0.5 0.5 0.5]);
hold off;
title('CoM Trajectory - World Frame');
xlabel('time (sec)');
ylabel('Position (m)');
legend('CoM_x','stanceFoot_x','swingFoot_x');
xlim([0 t_hist(end)]);
grid on;

ax = subplot(2,1,2);
plot(t_hist,F_stance_hist(:,2)' + c_hist(4,:),'g');
hold on;
plot(t_hist,F_stance_hist(:,2),'k.');
plot(t_hist,F_swing_hist(:,2),'k.','markerSize',1,'color',[0.5 0.5 0.5]);
hold off;
title('CoM Trajectory - World Frame');
xlabel('time (sec)');
ylabel('Position (m)');
legend('CoM_y','stanceFoot_y','swingFoot_y');
xlim([0 t_hist(end)]);
grid on;


figure(103);
plot(t_hist,E_y_hist);
xlabel('time (sec)');
ylabel('Orbital Energy (J/kg) (I think...)');
xlim([0 t_hist(end)]);
grid on;
