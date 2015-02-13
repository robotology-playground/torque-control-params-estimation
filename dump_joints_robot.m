%% Example to use this scripts
% This file is usefull to understand the functionality about all object
% included on this folder

%% Load object Motor
% Set all information about motor and robot when you would like to
% experiments
experiment_type = 'FrictionIdentification';
robot = Robot('iCubGenova03');
% Setup robot configuration:
% First variable
robot = robot.setConfiguration('root_link','true');

robot = robot.addMotor('leg','left','hip','roll');
robot = robot.addMotor('leg','right','hip','roll');
robot = robot.addMotor('leg','left','ankle','roll');
robot = robot.addMotor('leg','right','ankle','roll');

%robot = robot.addCoupledJoints('torso');

%% Configure computer
robot.configure('/Users/Raffaello/iit/codyco-superbuild');

%% Open Simulink
open([experiment_type '.slx']);