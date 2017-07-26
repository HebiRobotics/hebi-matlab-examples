%% HebiLookup
% *HebiLookup* represents the starting point of the API and provides
% ways to discover devices on the network and to establish communications
% with groups of modules.
%
%%
%
% <html>
% <table border=0>
%   <tr><td>Created</td><td>July 13, 2017</td></tr>
%   <tr><td>Last Update</td><td>July 26, 2017</td></tr>
%   <tr><td>API Version</td><td>hebi-matlab-1.0</td></tr>
%   <tr><td>Requirements</td><td>MATLAB 2013b or higher</td></tr>
% </table>
% </html>
%
% Copyright 2017 HEBI Robotics

%% Device Overview
% The first call to HebiLookup initializes a background process that can
% automatically discover devices using UDP broadcast messages. The default
% settings can be changed programmatically via set methods, or permanently  
% by modifying *hebi_config.m*
%
% Displaying HebiLookup provides an overview of all modules that were found
% on the network. The Family and Name columns show the user-settable names,
% and the Serial Number column shows the unique serial number of each 
% device.

% Display all devices found on the network
disp(HebiLookup);

%% Select Devices: Human Readable Names
% The preferred way for creating groups is to select devices based on their
% human readable *family* and *name*. Both parameters are user-settable and
% can be modified freely via the Scope GUI.
%
% A common convention is that the family identifies the robot (e.g.
% ' 2dof ') and that the name identifies the part (e.g. ' knee ').
%
% * The device order of the resulting group will match the order of the
%   specified names
% * Selection strings support the following wildcards: ' ? ' for a single
%   character and ' * ' for any number of characters
% * If more than one device meets a selection criteria, all matching 
%   devices will be added in alphabetical order

% Select devices by their user-settable family and name
family = '2dof';
names = {'base', 'knee'};
group = HebiLookup.newGroupFromNames(family, names);
display(group);

%% Select Devices: Hardware Serial Numbers
% Another common way for selecting devices is to refer to their unique
% hardware serial number. 
%
% * The resulting device order will match the order of the selection vector
% * Wilcards are not supported

% Select devices by their unique hardwre serial numbers
serials = {'X-00165', 'X-80040'};
group = HebiLookup.newGroupFromSerialNumbers(serials);
display(group);

%% Select Devices: Family Only
% This call creates a group of all devices that match the selected family 
% and sorts them in alphabetical order. This represents a simpler way to
% connect to an entire system at once, but it does require the names to be
% set to values that sort correctly. 
%
% For example, sortable names could look as follows
%
% * ' 1_base '
% * ' 2_shoulder '
% * ' 3_wrist '
%
% Wildcards are supported

% Select devices by family in alphabetical order
group = HebiLookup.newGroupFromFamily('X?-*');
display(group);

%% Debugging: Reset lookup
% Some cases such as connecting a network cable after the lookup has
% already started can result in devices not being found. This may be
% because available broadcast addresses are only set once on startup,
% resulting in new networks being ignored.
%
% In such cases, you can force a reinitialization manually using the code
% below.

% Clear state and reinitialize discovery on the default (all) interfaces
HebiLookup.clearModuleList();
HebiLookup.initOnce();

%% Debugging: Access Meta Data of all Devices
% Sometimes it may be necessary to programmatically access meta data that
% is not covered by the lookup. For such cases you can create a group of
% all modules (wildcard ' * ') and access the group info.

% Create a group of all modules on the network
group = HebiLookup.newGroupFromFamily('*');
info = group.getInfo();
display(info);




