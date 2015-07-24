
clear;
%% Example to use this scripts
% This file is usefull to understand the functionality about all object
% included on this folder

%% Load object Motor
% Set all information about your robot.
% - Name of robot
% - folder where you want save the data
% - codyco-superbuild folder (try to put "getenv('CODYCO_SUPERBUILD_ROOT')" )
robot = Robot('iCubGenova02', 'experiments', getenv('CODYCO_SUPERBUILD_ROOT'));
% Setup robot configuration:
% Variables:
% - worldRefFrame
% - robot_fixed
robot = robot.setReferenceFrame('root_link','true');

%% Add motors to test

% robot.joints = robot.getCoupledJoints('torso');

% robot.joints = robot.getJoint('l_hip_pitch');
% robot.joints = robot.getJoint('l_hip_roll');
% robot.joints = robot.getJoint('l_hip_yaw');
% robot.joints = robot.getJoint('l_knee');
% robot.joints = robot.getJoint('l_ankle_pitch');
% robot.joints = robot.getJoint('l_ankle_roll');
%  
% 
robot.joints = robot.getJoint('r_hip_pitch');
% robot.joints = robot.getJoint('r_hip_roll');
% robot.joints = robot.getJoint('r_hip_yaw');
% robot.joints = robot.getJoint('r_knee');
% robot.joints = robot.getJoint('r_ankle_pitch');
% robot.joints = robot.getJoint('r_ankle_roll');

%% Configure your computer
% Set all variables:
% - Name of joint list with your list of joints
% - If true, automatic set yarp namespace
robot.configure('JOINT_FRICTION','false');
robot.buildFolders();

%% Open Simulink
open('FrictionIdentification.mdl');