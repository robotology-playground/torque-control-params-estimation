%% Example to use this scripts
% This file is usefull to understand the functionality about all object
% included on this folder

%% Load object Motor
% Set all information about motor and robot when you would like to
% experiments
joint = CoupledJoints('iCubGenova03','torso');

%% Load from file measure of friction
joint = joint.loadIdleMeasure('idle');
%joint = joint.setFrictionToCenter();
%% Plot Friction
%Counter figures
if ~exist('counter','var')
    counter = 1;
end
%% FIGURE - Friction data and estimation
joint.savePictureFriction(counter);
counter = counter + 1;

clear counter;