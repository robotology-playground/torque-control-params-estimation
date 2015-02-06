%% Example to use this scripts
% This file is usefull to understand the functionality about all object
% included on this folder

%% Load object Motor
% Set all information about motor and robot when you would like to
% experiments
joint = Motor('experiments','iCubGenova01','leg','left','hip','roll');
joint = joint.setRatio(38.9,800);
%% Load from file measure of friction
joint = joint.loadIdleMeasure('idle-1F',10,0);
joint.friction = joint.friction.setToCenter();
joint = joint.loadRefFile('ref');
%% Plot Friction
%Counter figures
if ~exist('counter','var')
    counter = 1;
end
% FIGURE - Friction data and estimation
joint.friction.savePictureToFile(joint.path, counter);
counter = counter + 1;

% FIGURE - PWM vs Torque
hFig = figure(counter);
set(hFig, 'Position', [0 0 800 600]);
hold on
joint.plotKt();
grid;
hold off
joint.savePictureToFile(hFig,'PWMVsTorque');
%counter = counter + 1;

joint.friction  % print information about friction
clear counter;