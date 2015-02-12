%% Example to use this scripts
% This file is usefull to understand the functionality about all object
% included on this folder

%% Load object Motor
% Set all information about motor and robot when you would like to
% experiments
joint = CoupledJoints('iCubGenova03','torso');
joint = joint.setRatio(40,800);
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
counter = counter + 4 + 1;

name_ref = 'ref-11:36';
if exist([joint.path name_ref '.mat'],'file')
    joint = joint.loadRefMeasure(name_ref);
    % FIGURE - PWM vs Torque
    joint.savePictureKt(counter);
%     hFig = figure(counter);
%     set(hFig, 'Position', [0 0 800 600]);
%     hold on
%     joint.plotKt();
%     grid;
%     hold off
%     joint.savePictureToFile(hFig,'PWMVsTorque');
%     counter = counter + 1;
end

clear counter;