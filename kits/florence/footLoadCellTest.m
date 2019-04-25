% Load cell test with I/O board
%
% Plot force and center of pressure resutls for 4 load cells connected to
% I/O board during testing of a modular foot.
%
% For more information type:
%    help CommandStruct
%    help HebiGroup
%
% This script assumes you can create a group with 1 module.
%
% HEBI Robotics
% February 2019

%% Setup
clear *;
close all;
clf;

startup;
HebiLookup.initialize();

familyName = 'IO_BASIC_C';
moduleNames = 'IO_00096'; 
%moduleNames = 'IO_00090';

group = HebiLookup.newGroupFromNames( familyName, moduleNames );

%% Visualize analog inputs

% Start logging in the background
group.startLog( 'dir', 'logs' );

% Set duration of test
duration = 300; % [sec]
timer = tic();

% Initialize vectors for real time plotting
realTimeXPlot = [0];
realTimeYPlotFR = [0];
realTimeYPlotFL = [0];
realTimeYPlotBR = [0];
realTimeYPlotBL = [0];
realTimeX_COP = [0];
realTimeY_COP = [0];
i = 1;

% Initialize handles for real time plot figure and stop togglebutton
fig1Handle = figure(1);
set(fig1Handle,'Position',[100 100 1500 1000]);
movegui(fig1Handle,'center');

ButtonHandle = uicontrol('Style', 'togglebutton','String', 'Stop');
set(ButtonHandle,'Position',[20 20 150 75]);
set(ButtonHandle,'FontSize',24);

% Initialize subplots for force reading and center of pressure       
subplot(2,1,1);
ForcePlotHandle = plot(realTimeXPlot,realTimeYPlotFR, ...
                    realTimeXPlot,realTimeYPlotFL,...
                    realTimeXPlot,realTimeYPlotBR,...
                    realTimeXPlot,realTimeYPlotBL);

title('Real Time IO Board Load Cell Force Reading');
xlabel('Time');
ylabel('Sensor Force (Lbs)');
legend('front right', 'front left', 'back right', 'back left');
grid on;

subplot(2,1,2);
COPPlotHandle = plot(realTimeXPlot,realTimeX_COP, ...
                    realTimeXPlot,realTimeY_COP);

title('Real Time IO Board Load Cell COP');
xlabel('Time');
ylabel('Position (mm)');
legend('X COP', 'Y COP');
grid on;
ylim([-75,75]);

drawnow;
                     

while toc(timer) < duration
    
    drawnow;
    if (get(ButtonHandle,'Value')==1)
    disp('User Stopped Real Time Logging');
    break;
    end

    % Custom offset for force sensor load reading
    % For each load cell, this value should be 0.5 V minimum but will vary
    % Order as Front Left, Front Right, Back Left, Back Right
    
    %sensorOffset = [0.5,0.5,0.5,0.5]; %Default
    sensorOffset = [0.525,0.514,0.507,0.511]; %IO_00088
    %sensorOffset = [0.514,0.51,0.504,0.511]; %IO_00090
    
    % Read feedback
    footData = getFootData(group, sensorOffset);

    % Plot Force Reading in Real Time
    
    F = footData.Forces;
    
    realTimeXPlot = [realTimeXPlot i];
    realTimeYPlotFR = [realTimeYPlotFR F(4)];
    realTimeYPlotFL = [realTimeYPlotFL F(1)];
    realTimeYPlotBR = [realTimeYPlotBR F(3)];
    realTimeYPlotBL = [realTimeYPlotBL F(2)];
    i=i+1;
    
    ch = get(fig1Handle,'Children');
    ch = get(ch(5),'Children');
    
    set(ch(1),'Xdata',realTimeXPlot);
    set(ch(1),'Ydata',realTimeYPlotBL);
    set(ch(1),'Color','b');
    
    set(ch(2),'Xdata',realTimeXPlot);
    set(ch(2),'Ydata',realTimeYPlotBR);
    set(ch(2),'Color','r');
    
    set(ch(3),'Xdata',realTimeXPlot);
    set(ch(3),'Ydata',realTimeYPlotFL);
    set(ch(3),'Color','k');
    
    set(ch(4),'Xdata',realTimeXPlot);
    set(ch(4),'Ydata',realTimeYPlotFR);
    set(ch(4),'Color','g');
    

    % Plot COP in Real Time
    
    realTimeX_COP = [realTimeX_COP footData.COP(1)];
    realTimeY_COP = [realTimeY_COP footData.COP(2)];

    ch = get(fig1Handle,'Children');
    ch = get(ch(3),'Children');
    
    set(ch(1),'ydata',realTimeY_COP);
    set(ch(1),'xdata',realTimeXPlot);
    set(ch(1),'Color','r');
    
    set(ch(2),'ydata',realTimeX_COP);
    set(ch(2),'xdata',realTimeXPlot);
    set(ch(2),'Color','k');

end

disp('Stopped Logging!');

% Stop background logging
log = group.stopLogIO();

%% Plot the logged feedback

figure(3);

% output range from 0.5 to 4.5 v
% subtract offset of 0.5 v
% rated load is 100 lbs

% sensorForceFrontLeft = (log.a5 - 0.5)/4 * 100;
% sensorForceFrontRight = (log.a8 - 0.5)/4 * 100;
% sensorForceBackLeft = (log.a6 - 0.5)/4 * 100;
% sensorForceBackRight = (log.a7 - 0.5)/4 * 100;

%Translate voltage readings to force measurements using custom offsets
% sensorForceFrontLeft = (log.a5 - 0.525)/4 * 100;
% sensorForceFrontRight = (log.a8 - 0.514)/4 * 100;
% sensorForceBackLeft = (log.a6 - 0.507)/4 * 100;
% sensorForceBackRight = (log.a7 - 0.511)/4 * 100;
sensorForceFrontLeft = (log.a5 - sensorOffset(1))/4 * 100;
sensorForceFrontRight = (log.a8 - sensorOffset(2))/4 * 100;
sensorForceBackLeft = (log.a6 - sensorOffset(3))/4 * 100;
sensorForceBackRight = (log.a7 - sensorOffset(4))/4 * 100;

F = [sensorForceFrontLeft,sensorForceBackLeft,sensorForceBackRight,...
    sensorForceFrontRight];

F_threshold = 0.1; %artificial minimum force measurement in lbs
F = max(F,F_threshold); %set minimum threshold to ignore baseline noise

plot(log.time,F(:,4),log.time,F(:,1),log.time,F(:,3),log.time,F(:,2));

title('IO Board Load Cell Force Readings');
xlabel('Time (Sec)');
ylabel('Sensor Force (Lbs)');
legend('front right', 'front left', 'back right', 'back left');
grid on;

%% Calculate COP Center of Pressure

l = 52; %x axis distance mm
w = 46; %y axis distance mm

X_COP = l .*(-F(:,1)-F(:,2)+F(:,3)+F(:,4))./(F(:,1)+F(:,2)+F(:,3)+F(:,4));
Y_COP = w .*(F(:,1)-F(:,2)-F(:,3)+F(:,4))./(F(:,1)+F(:,2)+F(:,3)+F(:,4));

%% Apply low pass filter

Fs = 1000; %Sampling Frequency
fc = 1E-3; %Cutoff Frequency
wn = (2/Fs)*fc;

b = fir1(20,wn,'low',kaiser(21,10));
X_COP_Filt = filter(b,1,X_COP);
Y_COP_Filt = filter(b,1,Y_COP);

% X_COP_Filt = lowpass(X_COP',fc,Fs);
% Y_COP_Filt = lowpass(Y_COP',fc,Fs);

%% Plot COP Results

figure(4);

subplot(2,1,1);
plot(log.time,X_COP,log.time,Y_COP);

title('IO Board Load Cell COP');
xlabel('Time (Sec)');
ylabel('Position (mm)');
legend('X COP', 'Y COP');
grid on;
ylim([-75,75]);

subplot(2,1,2);
plot(log.time,X_COP_Filt,log.time,Y_COP_Filt);

title('IO Board Load Cell COP (Low Pass Filter)');
xlabel('Time (Sec)');
ylabel('Position (mm)');
legend('X COP', 'Y COP');
grid on;
ylim([-75,75]);














