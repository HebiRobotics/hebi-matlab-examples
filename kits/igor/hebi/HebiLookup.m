classdef (Sealed) HebiLookup
    % HebiLookup searches the network for modules and forms groups
    %
    %   HebiLookup periodically searches the <a href="matlab:disp(HebiLookup)">network</a> for modules. Available
    %   modules can be combined into groups, which are the basic way to
    %   send commands and retrieve feedback.
    %
    %   By default, the periodic lookup broadcasts on all local interfaces.
    %   If your network does not allow broadcasts or you want to only poll
    %   on certain ip address, you can manually set them, or modify the
    %   default behavior in the <a href="matlab:open('hebi_config')">config script</a>.
    %
    %   HebiLookup Methods (configuration):
    %   initialize                        - starts or resets the lookup process
    %   setLookupAddresses                - sets the lookup target address [ipv4]
    %   getLookupAddresses                - gets the lookup target address [ipv4]
    %   setLookupFrequency                - sets the lookup request rate [Hz]
    %   getLookupFrequency                - gets the lookup request rate [Hz]    
    %   setInitialGroupFeedbackFrequency  - sets the group feedback rate [Hz]
    %   setInitialGroupCommandLifetime    - sets the command lifetime [s]
    %   clearModuleList                   - clears device list (including stale)
    %
    %   HebiLookup Methods (group creation):
    %   newGroupFromNames                 - groups by user defined names
    %   newGroupFromFamily                - groups by family and sorts by name
    %   newGroupFromSerialNumbers         - groups by hardware serial numbers
    %   newGroupFromMacs                  - groups by hardware mac addresses
    %
    %   Generally there are two ways to address modules, either by their
    %   user-configurable names, or by their hardware identifiers such
    %   as mac address or serial number.
    %
    %   Example
    %      % Show devices on the network
    %      display(HebiLookup);
    %
    %   Examples
    %      % Create group using names
    %      family = 'Arm';
    %      names = {'Base'; 'Shoulder'; 'Elbow'};
    %      group = HebiLookup.newGroupFromNames(family, names);
    %
    %      % Create an alphabetically ordered group of all modules
    %      group = HebiLookup.newGroupFromFamily('*');
    %
    %      % Create group using serial numbers
    %      serials = {'X-00009'; 'X-00042'; 'X-00001'};
    %      group = HebiLookup.newGroupFromSerialNumbers(serials);\
    %
    %      % Create group using mac addresses
    %      macs = {'08:00:7F:9B:67:09'; '08:00:7F:50:BF:45'};
    %      group = HebiLookup.newGroupFromMacs(macs);
    %
    %   See also HebiGroup
    
    %   Copyright 2014-2018 HEBI Robotics, Inc.
    
    % Static API
    methods(Static)
        
        function this = initialize(varargin)
            %initialize starts or resets the lookup process
            %
            %   The lookup process gets initialized automatically the first
            %   time any method gets called, so it's not strictly required 
            %   for this method to be called manually.
            %
            %   However, this also serves as a way to reset the lookup
            %   configuration to its default parameters, which is useful in
            %   case the network setup has changed (e.g. plugged in an
            %   Ethernet cable to a new  network).
            %
            %   The default behavior can be modified in the <a href="matlab:open('hebi_config')">config script</a>
            %
            %   See also HebiLookup, setLookupAddresses,
            %   setLookupFrequency, setInitialGroupFeedbackFrequency,
            %   setInitialGroupCommandLifetime, clearModuleList
            HebiLookup.initOnce();
            this = HebiLookup.wrapper;
        end
        
        function this = setLookupAddresses(varargin)
            %setLookupAddresses sets the lookup target address [ipv4]
            %
            %   An address can be any unicast or broadcast ipv4 address.
            %   In most cases, it is best to let the system determine the
            %   correct addresses using '*'. However, if the network does
            %   not support broadcasting, or is otherwise restricted, it
            %   can be necessary to set addresses manually.
            %
            %   The default behavior can be changed in the <a href="matlab:open('hebi_config')">config script</a>.
            %
            %   Example
            %      % Custom addressing
            %      addresses = {
            %          '10.10.10.255'
            %          '192.168.0.255'
            %      };
            %      HebiLookup.setLookupAddresses(addresses);
            %
            %   See also HebiLookup, hebi_config
            javaMethod('setLookupAddresses', HebiLookup.className,  varargin{:});
            this = HebiLookup.wrapper;
        end
        
        function out = getLookupAddresses(varargin)
            %getLookupAddresses gets the lookup target address [ipv4]
            %
            %   See also HebiLookup, setLookupAddresses
            ips = javaMethod('getLookupAddresses', HebiLookup.className,  varargin{:});
            out = cell(ips);
        end
        
        function this = setLookupFrequency(varargin)
            % setLookupFrequency sets the lookup request rate in [Hz]
            %
            %   In addition to finding modules on the network, this rate
            %   also affects the rate at which info and (in default mode)
            %   gains of groups get updated.
            %
            %   These requests produce a relatively high load on modules,
            %   so the rate should be kept relatively low. For long running
            %   applications, it can make sense to disable (set to zero)
            %   lookups entirely in order to limit unnecessary traffic.
            %
            %   The default behavior can be changed in the <a href="matlab:open('hebi_config')">config script</a>.
            %
            %   See also HebiLookup, hebi_config
            javaMethod('setLookupFrequency', HebiLookup.className,  varargin{:});
            this = HebiLookup.wrapper;
        end
        
        function out = getLookupFrequency(varargin)
            %getLookupFrequency gets the lookup request rate in [Hz]
            %
            %   See also HebiLookup, setLookupFrequency
            out = javaMethod('getLookupFrequency', HebiLookup.className,  varargin{:});
        end
        
        function this = setInitialGroupFeedbackFrequency(varargin)
            % setInitialGroupFeedbackFrequency sets the group feedback rate [Hz]
            %
            %   This method affects the initial feedback request rate of
            %   newly created groups. It is a convenience method that
            %   avoids having to set the rate individually for every
            %   group.
            %
            %   The default behavior can be changed in the <a href="matlab:open('hebi_config')">config script</a>.
            %
            %   See also HebiLookup, HebiGroup.setFeedbackFrequency, hebi_config
            javaMethod('setInitialGroupFeedbackFrequency', HebiLookup.className,  varargin{:});
            this = HebiLookup.wrapper;
        end
        
        function this = setInitialGroupCommandLifetime(varargin)
            % setInitialGroupCommandLifetime sets the group command lifetime [s]
            %
            %   This method affects the initial command lifetime of newly
            %   created groups. It is a convenience method that avoids
            %   having to set the lifetime individually for every group.
            %
            %   The default behavior can be changed in the <a href="matlab:open('hebi_config')">config script</a>.
            %
            %   See also HebiLookup, HebiGroup.setCommandLifetime, hebi_config
            javaMethod('setInitialGroupCommandLifetime', HebiLookup.className,  varargin{:});
            this = HebiLookup.wrapper;
        end
        
        function this = clearModuleList(varargin)
            % clearModuleList clears list of available modules
            %
            %   This method clears the list of available modules that have
            %   been found by the lookup. Doing this can make sense when
            %   there are many modules on the network and the list has
            %   become cluttered with 'stale' modules.
            javaMethod('clearModuleList', HebiLookup.className,  varargin{:});
            this = HebiLookup.wrapper;
            
            % Add a pause so that the lookup has some
            % time to find modules
            config = hebi_config('HebiLookup');
            pause(config.initialNetworkLookupPause);
        end
        
        function this = clearGroups(varargin)
            % clearGroups disposes all existing groups
            %
            %   This method exists for advanced users. Generally, groups
            %   are disposed automatically, and this method does not need
            %   to be called.
            javaMethod('clearGroups', HebiLookup.className,  varargin{:});
            this = HebiLookup.wrapper;
        end
        
        function group = newGroupFromNames(varargin)
            % newGroupFromNames groups by user defined names
            %
            %   This method groups modules by their specified names and
            %   family. The family may be a single family for all modules,
            %   and may include wildcard characters (*,?).
            %
            %   Example
            %      % Group modules using a family wildcard
            %      family = 'Arm?';
            %      names = {
            %          'SA023'
            %          'SA032'
            %          'SA025'
            %      };
            %      group = HebiLookup.newGroupFromNames(family, names);
            %
            %   Example
            %      % Group modules with different families
            %      families = {
            %          'Arm1'
            %          'Arm2'
            %          'Arm3'
            %      };
            %      names = {
            %          'Base'
            %          'Shoulder'
            %          'Elbow'
            %      };
            %      group = HebiLookup.newGroupFromNames(families, names);
            %
            %   See also HebiLookup, HebiGroup
            group = HebiGroup(javaMethod('newGroupFromNames', HebiLookup.className,  varargin{:}));
        end
        
        function group = newGroupFromSerialNumbers(varargin)
            % newGroupFromSerialNumbers groups by hardware serial numbers
            %
            %   This method groups modules by their specified serial
            %   numbers.
            %
            %   Example
            %      serialNumbers = {
            %          'SA023'
            %          'SA032'
            %          'SA025'
            %      };
            %      group = HebiLookup.newGroupFromSerialNumbers(serialNumbers);
            %
            %   See also HebiLookup, HebiGroup
            group = HebiGroup(javaMethod('newGroupFromSerialNumbers', HebiLookup.className,  varargin{:}));
        end
        
        function group = newGroupFromFamily(varargin)
            % newGroupFromFamily groups by family and sorts by name
            %
            %   This method groups all modules that match the selected
            %   family, and orders them alphabetically by their name.
            %
            %   The family may include wildcard characters (*,?)
            %
            %   Example
            %      % Create group of all modules of any group
            %      group = HebiLookup.newGroupFromFamily('*');
            %
            %      % Create group of all modules in a certain group
            %      group = HebiLookup.newGroupFromFamily('Arm');
            %
            %   See also HebiLookup, HebiGroup
            group = HebiGroup(javaMethod('newGroupFromFamily', HebiLookup.className,  varargin{:}));
        end
        
        function group = newGroupFromMacs(varargin)
            % newGroupFromMacs groups by hardware mac addresses
            %
            %   This method groups modules by their specified mac
            %   addresses.
            %
            %   Example
            %      macs = {
            %          '08:00:7F:9B:67:09'
            %          '08:00:7F:50:BF:45'
            %      };
            %      group = HebiLookup.newGroupFromMacs(macs);
            %
            %   See also HebiLookup, HebiGroup
            group = HebiGroup(javaMethod('newGroupFromMacs', HebiLookup.className,  varargin{:}));
        end

    end
    
    % Static objects for delegation
    properties(Constant, Access = private, Hidden = true)
        className = HebiLookup.initOnce();
        wrapper = HebiLookup();
    end
    
    % Internal methods and deprecated API calls
    methods(Access = public, Static, Hidden = true)
        
        function fullName = initOnce()
            % Load library and config
            fullName = hebi_load('HebiLookup');
            config = hebi_config('HebiLookup');
            
            % Set up lookup parameters
            javaMethod('setLookupAddresses', fullName,  ...
                config.defaultLookupAddresses);
            javaMethod('setLookupFrequency', fullName,  ...
                config.defaultLookupFrequency);
            javaMethod('setInitialGroupFeedbackFrequency', fullName,  ...
                config.defaultInitialGroupFeedbackFrequency);
            javaMethod('setInitialGroupCommandLifetime', fullName,  ...
                config.defaultInitialGroupCommandLifetime);
            
            % Make sure all stale modules are cleared
            javaMethod('clearModuleList', fullName);
            
            % Add a pause on first call so that the lookup has some
            % time to find modules
            pause(config.initialNetworkLookupPause);
        end
        
        function group = newConnectedGroupFromName(varargin)
            % newConnectedGroupFromName groups by connectivity
            %
            %   This method groups all modules that are connected to the
            %   the module with the specified name, and orders them by
            %   their connectivity (proximal to distal).
            %
            %   Note that not all module types support this.
            %
            %   Example
            %      family = 'SnakeMonster';
            %      name = 'Base1';
            %      group = HebiLookup.newConnectedGroupFromName(family, name);
            %
            %   See also HebiLookup, HebiGroup
            group = HebiGroup(javaMethod('newConnectedGroupFromName', HebiLookup.className,  varargin{:}));
        end
        
        function group = newConnectedGroupFromSerialNumber(varargin)
            % newConnectedGroupFromSerialNumber groups by connectivity
            %
            %   This method groups all modules that are connected to the
            %   the module with the specified serial number, and orders them by
            %   their connectivity (proximal to distal).
            %
            %   Note that not all module types support this.
            %
            %   Example
            %      serial = 'SA023';
            %      group = HebiLookup.newConnectedGroupFromSerialNumber(serial);
            %
            %   See also HebiLookup, HebiGroup
            group = HebiGroup(javaMethod('newConnectedGroupFromSerialNumber', HebiLookup.className,  varargin{:}));
        end
        
        function group = newConnectedGroupFromMac(varargin)
            % newConnectedGroupFromMac groups by connectivity
            %
            %   This method groups all modules that are connected to the
            %   the module with the specified mac address, and orders them by
            %   their connectivity (proximal to distal).
            %
            %   Note that not all module types support this.
            %
            %   Example
            %      mac = '08:00:7F:9B:67:09';
            %      group = HebiLookup.newConnectedGroupFromMac(mac);
            %
            %   See also HebiLookup, HebiGroup
            group = HebiGroup(javaMethod('newConnectedGroupFromMac', HebiLookup.className,  varargin{:}));
        end
        
    end
    
    % Non-API Methods for MATLAB compliance
    methods(Access = public, Hidden = true)
        
        function this = HebiLookup
            % constructor
        end
        
        function disp(this)
            % custom display
            disp(javaObject(HebiLookup.className));
        end
        
    end
    
    % Non-API Static methods for MATLAB compliance
    methods(Access = public, Static, Hidden = true)
        
        function varargout = methods(varargin)
            instance = javaObject(HebiLookup.className);
            switch nargout
                case 0
                    methods(instance, varargin{:});
                    varargout = {};
                otherwise
                    varargout{:} = methods(instance, varargin{:});
            end
        end
        
        function varargout = fields(varargin)
            instance = javaObject(HebiLookup.className);
            switch nargout
                case 0
                    fields(instance, varargin{:});
                    varargout = {};
                otherwise
                    varargout{:} = fields(instance, varargin{:});
            end
        end
        
    end
    
end