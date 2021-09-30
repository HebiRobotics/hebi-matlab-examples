% Get analog and digital feedback from the touch screen of a mobile device, 
% log in the background, visualize live, and plot offline.
%
% This script assumes that you have run 'startup.m' in this folder.
%
% HEBI Robotics
% July 2018

%% Setup
clear *;
close all;
HebiLookup.initialize();

% Use Scope to change select a module and change the name and family to
% match the names below.  Following examples will use the same names.
familyName = 'HEBI';
deviceName = 'Mobile IO';

% Loop to keep trying to form the arm group.  Make sure the mobile device
% has the correct name / family to match the ones above and that it is on
% the same network as this computer.  You can check for this using Scope.
while true  
    try
        fprintf('Searching for phone Controller...\n');
        group = HebiLookup.newGroupFromNames( ...
                        familyName, deviceName );        
        disp('Phone Found.  Starting up');
        break;
    catch
        % If we failed to make a group, pause a bit before trying again.
        pause(1.0);
    end
end

mobileIO = HebiMobileIO( group );
mobileIO.setDefaults();

% Start logging in the background
group.startLog( 'dir', 'logs' ); 


%% Visualize Slider Input
disp('  Drag the sliders and press some buttons on the app screen...');

figure(1);
clf;

duration = 15; % [sec]
timer = tic();
while toc(timer) < duration
    
    % Get Feedback
    mobileIO.update();  
    fbkIO = mobileIO.getFeedbackIO();
    
    % Digital feedback (buttons)
    bar( [fbkIO.b1 fbkIO.b2 fbkIO.b3 fbkIO.b4 ...
          fbkIO.b5 fbkIO.b6 fbkIO.b7 fbkIO.b8], 'r' );
    hold on;
    
    % Analog feedback (sliders)
    bar( [fbkIO.a1 fbkIO.a2 fbkIO.a3 fbkIO.a4 ...
          fbkIO.a5 fbkIO.a6 fbkIO.a7 fbkIO.a8], 'b' );
    hold off;
    
    % Format plot
    yAxisMaxLim = 1;
    yAxisMinLim = -1;
    ylim([yAxisMinLim yAxisMaxLim]);
    title('Digital Inputs (red) and Analog Inputs (blue)');
    ylabel('[-1 to 1]');
    grid on;
    drawnow;
    
end

disp('  All done!');

% Stop background logging
logIO = group.stopLogIO();  

%% Plot the logged feedback
figure(101);
subplot(2,1,1);
plot( logIO.time, logIO.a1 );
hold on;
plot( logIO.time, logIO.a2 );
plot( logIO.time, logIO.a3 );
plot( logIO.time, logIO.a4 );
plot( logIO.time, logIO.a5 );
plot( logIO.time, logIO.a6 );
plot( logIO.time, logIO.a7 );
plot( logIO.time, logIO.a8 );
hold off;

title('Analog Inputs');
xlabel('time (sec)');
ylabel('[-1 to 1]');
ylim([-1.1 1.1]);
legend( strsplit(num2str(1:8)) );
grid on;

subplot(2,1,2);
plot( logIO.time, logIO.b1, '.' );
hold on;
plot( logIO.time, logIO.b2, '.' );
plot( logIO.time, logIO.b3, '.' );
plot( logIO.time, logIO.b4, '.' );
plot( logIO.time, logIO.b5, '.' );
plot( logIO.time, logIO.b6, '.' );
plot( logIO.time, logIO.b7, '.' );
plot( logIO.time, logIO.b8, '.' );
hold off;

title('Digital Inputs');
xlabel('time (sec)');
ylabel('[0 or 1]');
ylim([-0.1 1.1]);
legend( strsplit(num2str(1:8)) );
grid on;

