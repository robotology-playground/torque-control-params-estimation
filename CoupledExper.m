%% Example to use this scripts
% This file is usefull to understand the functionality about all object
% included on this folder

%% Load object Motor
% Set all information about motor and robot when you would like to
% experiments
joint = CoupledJoints('experiments', 'iCubGenova03', 'torso');

%% Load from file measure of friction
joint = joint.loadIdleMeasure('idle',5,100);
joint = joint.loadRefFile('ref');

%% Save information in file
joint.saveToFile();
%joint.saveControlToFile();

%% Plot Friction
joint.plotFriction();
