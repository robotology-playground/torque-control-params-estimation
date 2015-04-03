%% Example to use this scripts
% This file is usefull to understand the functionality about all object
% included on this folder
%% Load object Motor
% Set all information about your robot.
% - Name of robot
% - folder where you want save the data
% - codyco-superbuild folder (try to put "getenv('CODYCO_SUPERBUILD_ROOT')" )
robot = Robot('iCubGenova02', 'experiments', '/Users/Raffaello/iit/codyco-superbuild');

%% Add motors to test
% robot.joints = robot.getJoint('r_elbow');
% or you can add a coupled joints
robot.joints = robot.getCoupledJoints('r_shoulder');
% finally you can add all joints with
% robot = robot.addParts('left_leg');

%% Load from file measure of friction
% load data from name
% if name start with "idle" the file will be loaded for study friction
% otherwise, if the name start with "ref" the file will be loaded for study
% motor coefficient.
robot = robot.loadData('idle');
robot = robot.loadData('ref');

%% Estimate and plot Kt
% Plot and save all information about motor
robot.plotAndPrintAllData();

%% Collect all data.txt in single folder
robot.appendAllData('ParametersJTC');