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
    allTypes = allModules.getInfo.mechanicalType;

    display('Setting X5-1 Module Gains...');
    gainGroupNames = allNames(strcmp(allTypes,'X5-1'));
    if length(gainGroupNames) > 0
        gainGroup = HebiLookup.newGroupFromNames('*',gainGroupNames);
        setGains_X5_1( gainGroup );
        gainGroup.set('SpringConstant',70,'persist',true);
    else
        display('No X5-1 Modules Found!');
    end

    display('Setting X5-4 Module Gains...');
    gainGroupNames = allNames(strcmp(allTypes,'X5-4'));
    if length(gainGroupNames) > 0
        gainGroup = HebiLookup.newGroupFromNames('*',gainGroupNames);
        setGains_X5_4( gainGroup );
        gainGroup.set('SpringConstant',130,'persist',true);
    else
        display('No X5-4 Modules Found!');
    end

    display('Setting X5-9 Module Gains...');
    gainGroupNames = allNames(strcmp(allTypes,'X5-9'));
    if length(gainGroupNames) > 0
        gainGroup = HebiLookup.newGroupFromNames('*',gainGroupNames);
        setGains_X5_9( gainGroup );
        gainGroup.set('SpringConstant',130,'persist',true);
    else
        display('No X5-9 Modules Found!');
    end
    
    display('Setting X8-3 Module Gains...');
    gainGroupNames = allNames(strcmp(allTypes,'X8-3'));
    if length(gainGroupNames) > 0
        gainGroup = HebiLookup.newGroupFromNames('*',gainGroupNames);
        setGains_X8_3( gainGroup );
        gainGroup.set('SpringConstant',130,'persist',true);
    else
        display('No X8-3 Modules Found!');
    end
end