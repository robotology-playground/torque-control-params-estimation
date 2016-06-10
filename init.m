%% Example to use this scripts
% This file is usefull to understand the functionality about all object
% included on this folder

clear;
%% Load object Motor
% Set all information about your robot.
% - Name of robot
% - folder where you want save the data
% - codyco-superbuild folder (try to put "getenv('CODYCO_SUPERBUILD_ROOT')" )
% - optional: name of the yarpWholeBodyInterface configuration file.
%              Default: yarpWholeBodyInterface.ini 
% - optional: name of the build directory 
%              Default: build 

% if you have to set some additional environmental variables, set it here.
% e.g. this script will use yarp to check namespace existance. 
% If yarp is not in PATH, please set it here
% path = getenv('PATH');
% path = strcat(path, ':', '/path/to/yarp/bin');
% setenv('PATH', path);

robotName = getenv('YARP_ROBOT_NAME');
% you can overwrite the robot name by simply setting it after this line
% robotName = 'anotherRobot';

% you can overwrite the superbuild directory by simply setting it after this line
codycoSuperbuildDir = getenv('CODYCO_SUPERBUILD_ROOT');
% codycoSuperbuildDir = '/path/to/superbuild/';

% Directory where experimental data will be saved.
outputDir = 'experiments';

%Note: it assumes the build directory is called 'build'
robot = Robot(robotName, outputDir, codycoSuperbuildDir);
% Setup robot configuration:
% Variables:
% - worldRefFrame
% - robot_fixed
robot = robot.setReferenceFrame('root_link','true');

%% Add motors to test
% robot.joints = robot.getCoupledJoints('torso');

% robot.joints = robot.getCoupledJoints('r_shoulder');
% robot.joints = robot.getJoint('r_elbow');
robot.joints = robot.getJoint('l_hip_pitch');

% robot.joints = robot.getCoupledJoints('l_shoulder');
% robot.joints = robot.getJoint('l_elbow');
% robot.joints = robot.getJoint('l_wrist_prosup');

% robot.joints = robot.getJoint('l_hip_pitch');
% robot.joints = robot.getJoint('l_hip_roll');
% robot.joints = robot.getJoint('l_hip_yaw');
% robot.joints = robot.getJoint('l_knee');
% robot.joints = robot.getJoint('l_ankle_pitch');
% robot.joints = robot.getJoint('l_ankle_roll');
%  
% 
% robot.joints = robot.getJoint('r_hip_pitch');
% robot.joints = robot.getJoint('r_hip_roll');
% robot.joints = robot.getJoint('r_hip_yaw');
% robot.joints = robot.getJoint('r_knee');
% robot.joints = robot.getJoint('r_ankle_pitch');
% robot.joints = robot.getJoint('r_ankle_roll');

% robot.joints = [robot.joints robot.getJoint('l_hip_roll')];
% robot.joints = [robot.joints robot.getJoint('l_hip_yaw')];
% robot.joints = [robot.joints robot.getCoupledJoints('torso')];
% robot.joints = [robot.joints robot.getCoupledJoints('l_shoulder')];

%% Configure your computer
% Set all variables:
% - Name of joint list with your list of joints
% - If true, automatic set yarp namespace
robot.configure('JOINT_FRICTION','false');
robot.buildFolders();

%% Open Simulink
open('FrictionIdentification.mdl');
