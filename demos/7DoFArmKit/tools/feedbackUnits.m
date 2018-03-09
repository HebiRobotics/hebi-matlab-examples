function [ feedbackUnits ] = feedbackUnits( feedbackField )
%FEEDBACKUNITS Return units of a given feedback type in a log file

    switch feedbackField
        case {'position','positionCmd','motorPosition','deflection'}
           feedbackUnits = 'rad';   
           
        case {'velocity','velocityCmd','motorVelocity','deflectionVelocity'}
           feedbackUnits = 'rad/sec';  
           
        case {'effort','effortCmd','innerEffortCmd'}
           feedbackUnits = 'Nm'; 
           
        case {'time','pcRxTime','pcTxTime','hwRxTime','hwTxTime'}
           feedbackUnits = 'sec'; 
           
        case {'pwmCmd'}
           feedbackUnits = '-1 to 1'; 
           
        case {'accelX','accelY','accelZ'}
           feedbackUnits = 'm/sec^2';
           
        case {'gyroX','gyroY','gyroZ'}
           feedbackUnits = 'rad/sec';   
           
        case {'motorTemperature','windingTemperature','ambientTemperature'...
              'processorTemperature','actuatorTemperature'}  
           feedbackUnits = 'deg-C';
           
        case {'motorCurrent','windingCurrent'}
           feedbackUnits = 'A';  
           
        case {'voltage'}
           feedbackUnits = 'V';
           
        case {'ledR','ledRG','ledB'}
           feedbackUnits = '0-1';    
           
        otherwise
           feedbackUnits = '';     
    end


end

