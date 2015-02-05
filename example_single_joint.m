%% Example to use this scripts
% This file is usefull to understand the functionality about all object
% included on this folder

%% Load object Motor
% Set all information about motor and robot when you would like to
% experiments
joint = Motor('experiments','iCubGenova03','arm','left','elbow');

%% Load from file measure of friction
joint = joint.loadIdleMeasure('idle',10);
