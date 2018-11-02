% Kinematics Logged Data Analysis
%
% Features:      Loads an example log file and kinematics description for 
%                plotting end-effector error and tracking.
%
% Requirements:  MATLAB 2013b or higher
%
% Author:        Dave Rollinson
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
% Date:          Oct 2018

% Copyright 2017-2018 HEBI Robotics

% The local directory of this file to make sure that the XML / HRDF files 
% load correctly.  
%
% NOTE:
% If you run this script using Evaluate Selection (F9) or Evaluate Current 
% Section (CTRL-ENTER) this will not work unless you are working from the 
% directory that has this script.  
localDir = fileparts(mfilename('fullpath'));
if isempty(localDir) && (isunix || ismac)
    localDir = '.';
end
    

% Load example log file and kinematics.  You can replace these with data
% from your own arm.  You want to make sure that the HRDF file matches the
% arm that used in the logged data :-).
hebilog = HebiUtils.loadGroupLog( [localDir '/logs/3-DoF_arm_example_log'] );
kin = HebiKinematics( [localDir '/hrdf/3-DoF_arm'] );

kinematics_analysis( hebilog, kin ); 

