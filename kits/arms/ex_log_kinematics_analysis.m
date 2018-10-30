% Load a log file and kinematics description for plotting end effector
% errror and tracking.
%
% Dave Rollinson
% Sep 2018

% Load example log file and kinematics.  You can replace these with data
% from your own arm.  You want to make sure that the HRDF file matches the
% arm that used in the logged data :-).
hebilog = HebiUtils.loadGroupLog('logs/3-DoF_arm_example_log');
kin = HebiKinematics('hrdf/3-DoF_arm');

kinematics_analysis( hebilog, kin ); 
