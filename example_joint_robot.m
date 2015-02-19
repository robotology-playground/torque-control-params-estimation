%% Example to use this scripts
% This file is usefull to understand the functionality about all object
% included on this folder

%% Load object Motor
% Set all information about motor and robot when you would like to
% experiments
robot = Robot('iCubGenova01');
robot = robot.addMotor('leg','left','hip','roll');

%% Load from file measure of friction
[robot, counter] = robot.plotAndPrintAllData('idle-20150206-19:07');

%% Estimate and plot Kt
[robot, counter] = robot.plotAndPrintAllData('ref-20150213-17:47', counter);