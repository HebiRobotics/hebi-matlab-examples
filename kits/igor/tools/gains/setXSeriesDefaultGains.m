function [ ] = setXSeriesDefaultGains( familyName )
    % PUTTING DEFAULT GAINS ON X5 ACTUATORS
    %
    % Dave Rollinson
    % May 2016

    fprintf('Clearing Module List...');
    HebiLookup.clearModuleList;
    pause(1.0);
    fprintf('DONE!\n');

    allModules = HebiLookup.newGroupFromFamily(familyName);

    % Make cell arrays, even if only a single module
    allNames = allModules.getInfo.name;
    allMACs = allModules.getInfo.macAddress;
    allTypes = allModules.getInfo.mechanicalType;

    disp('Setting X5-1 Module Gains...');
    gainGroupMACs = allMACs(strcmp(allTypes,'X5-1'));
    if ~isempty(gainGroupMACs)
        gainGroup = HebiLookup.newGroupFromMacs(gainGroupMACs);
        setGains_X5_1( gainGroup );
    else
        disp('No X5-1 Modules Found!');
    end

    disp('Setting X5-4 Module Gains...');
    gainGroupMACs = allMACs(strcmp(allTypes,'X5-4'));
    if ~isempty(gainGroupMACs)
        gainGroup = HebiLookup.newGroupFromMacs(gainGroupMACs);
        setGains_X5_4( gainGroup );
    else
        disp('No X5-4 Modules Found!');
    end

    disp('Setting X5-9 Module Gains...');
    gainGroupMACs = allMACs(strcmp(allTypes,'X5-9'));
    if ~isempty(gainGroupMACs)
        gainGroup = HebiLookup.newGroupFromMacs(gainGroupMACs);
        setGains_X5_9( gainGroup );
    else
        disp('No X5-9 Modules Found!');
    end
    
    disp('Setting X8-3 Module Gains...');
    gainGroupMACs = allMACs(strcmp(allTypes,'X8-3'));
    if ~isempty(gainGroupMACs)
        gainGroup = HebiLookup.newGroupFromMacs(gainGroupMACs);
        setGains_X8_3( gainGroup );
    else
        disp('No X8-3 Modules Found!');
    end
    
    disp('Setting X8-9 Module Gains...');
    gainGroupMACs = allMACs(strcmp(allTypes,'X8-9'));
    if ~isempty(gainGroupMACs)
        gainGroup = HebiLookup.newGroupFromMacs(gainGroupMACs);
        setGains_X8_9( gainGroup );
    else
        disp('No X8-9 Modules Found!');
    end
end