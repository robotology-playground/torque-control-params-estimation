%% Example to use this scripts
% This file is usefull to understand the functionality about all object
% included on this folder

%% Load object Motor
% Set all information about motor and robot when you would like to
% experiments
joint = Motor('experiments','iCubGenova03','arm','right','elbow');
% Set number of joint you want control and read
joint = joint.setPart('number_joint',1);

%% Set variable environment
setenv('YARP_DATA_DIRS', '/Users/Raffaello/iit/codyco-superbuild/build/install/share/codyco');
localName = 'simulink_joint_friction';
setenv('YARP_ROBOT_NAME', joint.robot);

ROBOT_DOF = 1;
robotName = 'icub';
Ts = 0.01;
name = 'idle';

plot = struct;
plot.x = [-1500 1500];
plot.y = [-10 10];

%% Load from file measure of friction
joint = joint.loadIdleMeasure(name,10);

%% Plot Friction
hFig = figure(1);
set(hFig, 'Position', [0 0 800 600]);
hold on
grid;
%friction_data = joint.friction.setToCenter();
friction_data = joint.friction;
friction_data.plotFriction();
friction_data.plotFrictionModel();
clear friction_data;
hold off
% print information about friction
joint.friction

% %% Save image
% currentFolder = pwd;
% cd(joint.path);
% saveas(hFig,[name '.fig'],'fig');
% saveas(hFig,[name '.png'],'png');
% cd(currentFolder);
% clear currentFolder;