%% Example to use this scripts
% This file is usefull to understand the functionality about all object
% included on this folder

%% Load object Motor
% Set all information about your robot.
% - Name of robot
% - folder where you want save the data
% - codyco-superbuild folder (try to put "getenv('CODYCO_SUPERBUILD_ROOT')" )
% - optional: name of the yarpWholeBodyInterface configuration file.
%              Default: yarpWholeBodyInterface.ini 
% - optional: name of the build directory 
%              Default: build 
robot = Robot('iCubGenova01', 'experiments', '/usr/local/src/robot/codyco-superbuild');
% Setup robot configuration:
% Variables:
% - worldRefFrame
% - robot_fixed
robot = robot.setReferenceFrame('root_link','true');

%% Add motors to test
robot.joints = [robot.joints robot.getJoint('l_hip_roll')];
robot.joints = [robot.joints robot.getJoint('l_hip_yaw')];
% robot.joints = [robot.joints robot.getCoupledJoints('torso')];
% robot.joints = [robot.joints robot.getCoupledJoints('l_shoulder')];

%% Configure your computer
% Set all variables:
% - Name of joint list with your list of joints
% - If true, automatic set yarp namespace
robot.configure('JOINT_FRICTION','false');
robot.buildFolders();

%% Open Simulink
open('FrictionIdentification.slx');