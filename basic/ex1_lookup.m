% HebiLookup represents the starting point of the API and provides
% ways to discover devices on the network and to establish communications
% with groups of modules, i.e., form a HebiGroup.
%
% Created groups represent the basic way to send commands and retrieve 
% feedback.  They provide convenient ways to deal with modules, and handle  
% high-level issues such as data synchronization and logging.
%
% (hint: individual sections can be run using ctrl+enter)
% 
% Requirements:  MATLAB 2013b or higher
%
% Author:        Florian Enner
% Created:       13 July, 2017
% API:           hebi-matlab-1.0
%
% Copyright 2017 HEBI Robotics

%% Display all devices that are on the network
% The first call to HebiLookup initializes a background process that can
% automatically discover modules using UDP broadcast messages. The default
% settings can be changed programmatically via set methods, or permanently  
% by modifying hebi_config.m
disp(HebiLookup);

%% Reset lookup
% Available broadcast addresses are only set once on startup. If new
% network interfaces get added after the first call (e.g. connected to
% Wi-Fi), a reinitialization can be forced programmatically.
HebiLookup.clearModuleList(); % clears stale modules
HebiLookup.initOnce(); % re-initializes discovery to default values

%% Get meta data of all modules on the network
% If meta data needs to be accessed programmatically, you can create a
% group of all modules and call getInfo().
group = HebiLookup.newGroupFromFamily('*');
info = group.getInfo();
display(info);

%% Create group using serial numbers
serials = {'X-00165', 'X-80040'};
group = HebiLookup.newGroupFromSerialNumbers(serials);
display(group);

%% Create group using human readable names
family = '2dof';
names = {'base', 'knee'};
group = HebiLookup.newGroupFromNames(family, names);
display(group);

%% Create group using only the family
% This call provides a way to quickly connect to an entire robot by 
% grouping all modules in a specified family, and by sorting them
% alphabetically.
% Note that the family name supports wildcards, i.e., '?' for a single
% unknown character, and '*' for zero or more characters.
group = HebiLookup.newGroupFromFamily('X?-*');
display(group);

