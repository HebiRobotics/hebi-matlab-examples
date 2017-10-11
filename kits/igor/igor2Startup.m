%%
%run_igor
clear *;
close all;

%%
startup;
HebiLookup.setLookupAddresses('10.10.1.255');
pause(1.0);

moduleNames = { 'wheel1', 'wheel2', ...
                'hip1', 'knee1', ...
                'hip2', 'knee2', ...
                'base1', 'shoulder1', 'elbow1', 'wrist1', ...
                'base2', 'shoulder2', 'elbow2', 'wrist2', ...
                'camTilt' };

while true
    try
        disp('Searching for modules');
        robotGroup = HebiLookup.newGroupFromNames('Igor II',moduleNames);
        
        %loop until all modules are in application
        while true
            moduleInfo = robotGroup.getInfo();
            inBootloader = strcmp('bootloader', moduleInfo.firmwareMode);

            %if any modules are in bootloader just try to boot all of them
            if (any(inBootloader))
                robotGroup.send('boot',true);
                pause(0.5);
            else
                break;
            end       
        end
        break;
       
    catch
        %keep going, there are still modules missing
    end
    
    pause(0.1)
end

%%

disp('Modules found!');
robotGroup.send('led','m');

%%
%try joystick

while true        
    try
        disp('Searching for joystick');
        joy = HebiJoystick(1);
        break;
    catch
        %do nothing
    end
    
    robotGroup.send('led','w');
    pause(0.1);
    robotGroup.send('led','m');
    pause(0.1);
end

disp('Joystick found!');
robotGroup.send('led',[]);
%%
%end
[axes, buttons, povs] = read(joy);
START_STOP_BUTTON = 11;
 
%%
while true

    try
        [axes, buttons, povs] = read(joy);
    catch
       
        while true        
            try
                disp('Searching for joystick');
                joy = HebiJoystick(1);
                break;
            catch
                %do nothing
            end

            robotGroup.send('led','w');
            pause(0.1);
            robotGroup.send('led','m');
            pause(0.1);
        end
        disp('Joystick found!');
        robotGroup.send('led','m');
    end
    
    START_STOP_BUTTON = 11;
    
    if buttons(START_STOP_BUTTON)
        pause(0.5);
        robotGroup.send('led',[]);
        
        try
        [axes, buttons, povs] = read(joy);
        disp('Running igor2Demo');
        igor2Demo_w_Camera;
        pause(0.5);
        catch
            %If igor2Demo exits with an error, it might be the joystick
            while true        
                try
                    disp('Searching for joystick');
                    joy = HebiJoystick(1)
                    break;
                catch
                    %do nothing
                end

                robotGroup.send('led','w');
                pause(0.1);
                robotGroup.send('led','m');
                pause(0.1);
            end  
            disp('Joystick found!');
            
        end
        
    end
    
    %Default to magenta for "ready"
    robotGroup.send('led','m');

    pause(0.1);

end
