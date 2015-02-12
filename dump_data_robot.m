%% Example to use this scripts
% This file is usefull to understand the functionality about all object
% included on this folder

%% Load object Motor
% Set all information about motor and robot when you would like to
% experiments
robot = Robot('iCubGenova03');
robot = robot.setConfiguration('root_link','true');

robot = robot.addMotor('leg','left','hip','roll');
robot = robot.addMotor('leg','right','hip','roll');
robot = robot.addMotor('leg','left','ankle','roll');
robot = robot.addMotor('leg','right','ankle','roll');

%% Configure computer
%robot.configure('/Users/Raffaello/iit/codyco-superbuild');

%% Configure experiment
robot.setupExperiment('idle');

%% Open Simulink
open('FrictionIdentification.slx');