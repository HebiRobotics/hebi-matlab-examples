function [] = FeedbackStruct()
    % FeedbackStruct shows feedback coming from groups of modules.
    %
    %   Feedback structs are instantiated by various API calls and can't be
    %   instantiated directly by users. Depending on the requested
    %   feedback, the returned struct may provide a different 'view' on the
    %   data, i.e., return a specific subset of sensor feedback and states.
    %
    %   A list of currently available views and their fields is below. You can
    %   find more detailed information about feedback fields at <a href="http://docs.hebi.us">docs.hebi.us</a>.
    %
    %   'Simple' (Default)
    %
    %                   time [s]
    %            positionCmd [rad]
    %            velocityCmd [rad/s]
    %              effortCmd [Nm]
    %               position [rad]
    %               velocity [rad/s]
    %                 effort [Nm]
    %                 accelX [m/s^2]
    %                 accelY [m/s^2]
    %                 accelZ [m/s^2]
    %                  gyroX [rad/s]
    %                  gyroY [rad/s]
    %                  gyroZ [rad/s]
    %           motorCurrent [A]
    %         windingCurrent [A]
    %     ambientTemperature [C]
    %     windingTemperature [C]
    %                voltage [V]
    %
    %
    %   'Full' ('Simple' with extra states)
    %
    %                     time [s]
    %                 pcRxTime [s]
    %                 pcTxTime [s]
    %                 hwRxTime [s]
    %                 hwTxTime [s]
    %                 sequence
    %               rxSequence
    %              positionCmd [rad]
    %              velocityCmd [rad/s]
    %                effortCmd [Nm]
    %                   pwmCmd
    %           innerEffortCmd [Nm]
    %                 position [rad]
    %                 velocity [rad/s]
    %                   effort [Nm]
    %               deflection [rad]
    %            motorPosition [rad]
    %            motorVelocity [rad/s]
    %       deflectionVelocity [rad/s]
    %                   accelX [m/s^2]
    %                   accelY [m/s^2]
    %                   accelZ [m/s^2]
    %                    gyroX [rad/s]
    %                    gyroY [rad/s]
    %                    gyroZ [rad/s]
    %             orientationW
    %             orientationX
    %             orientationY
    %             orientationZ
    %             motorCurrent [A]
    %           windingCurrent [A]
    %         motorTemperature [C]
    %       ambientTemperature [C]
    %     processorTemperature [C]
    %      actuatorTemperature [C]
    %       windingTemperature [C]
    %                  voltage [V]
    %                 pressure [kPa]
    %         resetButtonState [HebiEnum]
    %               mStopState [HebiEnum]
    %         commandLockState [HebiEnum]
    %         temperatureState [HebiEnum]
    %       positionLimitState [HebiEnum]
    %       velocityLimitState [HebiEnum]
    %         effortLimitState [HebiEnum]
    %                     ledR [0-1]
    %                     ledG [0-1]
    %                     ledB [0-1]
    %                     ledA [0-1]
    %
    %   'Mobile' (Special view for mobile devices)
    %
    %                      time [s]
    %                  pcRxTime [s]
    %                  pcTxTime [s]
    %                  hwRxTime [s]
    %                  hwTxTime [s]
    %                    accelX [m/s^2]
    %                    accelY [m/s^2]
    %                    accelZ [m/s^2]
    %                     gyroX [rad/s]
    %                     gyroY [rad/s]
    %                     gyroZ [rad/s]
    %             magnetometerX [T]
    %             magnetometerY [T]
    %             magnetometerZ [T]
    %                  altitude [m]
    %              orientationW
    %              orientationX
    %              orientationY
    %              orientationZ
    %               gpsLatitude [deg]
    %              gpsLongitude [deg]
    %               gpsAltitude [m]
    %                gpsHeading [deg]
    %     gpsHorizontalAccuracy [m]
    %       gpsVerticalAccuracy [m]
    %              gpsTimestamp [s]
    %            arOrientationW
    %            arOrientationX
    %            arOrientationY
    %            arOrientationZ
    %               arPositionX [m]
    %               arPositionY [m]
    %               arPositionZ [m]
    %                 arQuality [HebiEnum]
    %              batteryLevel [%]
    %
    %
    %   'IO' (Special view for IO devices)
    %
    %         time [s]
    %     pcRxTime [s]
    %     pcTxTime [s]
    %     hwRxTime [s]
    %     hwTxTime [s]
    %           a1
    %           a2
    %           a3
    %           a4
    %           a5
    %           a6
    %           a7
    %           a8
    %           b1
    %           b2
    %           b3
    %           b4
    %           b5
    %           b6
    %           b7
    %           b8
    %           c1
    %           c2
    %           c3
    %           c4
    %           c5
    %           c6
    %           c7
    %           c8
    %           d1
    %           d2
    %           d3
    %           d4
    %           d5
    %           d6
    %           d7
    %           d8
    %           e1
    %           e2
    %           e3
    %           e4
    %           e5
    %           e6
    %           e7
    %           e8
    %           f1
    %           f2
    %           f3
    %           f4
    %           f5
    %           f6
    %           f7
    %           f8 
    %
    %   Examples:
    %       % Read simple feedback from a group of two dummy modules
    %       group = HebiUtils.newImitationGroup(2);
    %       fbk = group.getNextFeedback();
    %       disp(fbk.time);
    %
    %       % Read feedback from an I/O device
    %       ioFbk = group.getNextFeedbackIO();
    %
    %       % Simultaneously read I/O and mobile feedback from a mobile device
    %       ioFbk = group.getNextFeedbackIO();
    %       mobileFbk = group.getNextFeedbackMobile();
    %       group.getNextFeedback(ioFbk, mobileFbk); % updates structs passed in
    %
    %   See also HebiLookup, HebiGroup, HebiEnum, CommandStruct, HebiGroup.getNextFeedback,
    %   HebiGroup.stopLog, HebiUtils.convertGroupLog.
    
    %   Copyright 2014-2019 HEBI Robotics, Inc.
    error('FeedbackStruct cannot be instantiated directly');
end
