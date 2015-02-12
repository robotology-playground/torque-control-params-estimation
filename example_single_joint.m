%% Example to use this scripts
% This file is usefull to understand the functionality about all object
% included on this folder

%% Load object Motor
% Set all information about motor and robot when you would like to
% experiments
joint = Motor('experiments','iCubGenova01','leg','left','hip','roll');
joint = joint.setRatio(40,8000);
%% Load from file measure of friction
joint = joint.loadIdleMeasure('idle-20150211-11:00');
%joint = joint.setFrictionToCenter();
%% Plot Friction
%Counter figures
if ~exist('counter','var')
    counter = 1;
end
%% FIGURE - Friction data and estimation
joint.savePictureFriction(counter);
counter = counter + 1;

%% FIGURE - Noise on data
hFig = figure(counter);
set(hFig, 'Position', [0 0 800 600]);
hold on
joint.friction.plotNoise();
grid;
hold off
joint.savePictureToFile(hFig,'Noise');
counter = counter + 1;

disp(joint.friction);  % print information about friction

%% Estimate and plot Kt
name_ref = 'ref';
if exist([joint.path name_ref '.mat'],'file')
    joint = joint.loadRefMeasure(name_ref);
    % FIGURE - PWM vs Torque
    joint.savePictureKt(counter);
end
clear name_ref hFig;

%% Save information to file
joint.saveToFile();
joint.saveControlToFile();

clear counter;