%% Example to use this scripts
% This file is usefull to understand the functionality about all object
% included on this folder

%% Load object Motor
% Set all information about motor and robot when you would like to
% experiments
joint = Motor('experiments','iCubGenova03','arm','right','elbow');
% Set number of joint you want control and read
joint = joint.setPart('number_joint',1);
% List to load in ---
disp(joint.getWBIlist());
% Start Control board dumper
disp(joint.getControlBoardCommand());

%% Set variable environment
setenv('YARP_DATA_DIRS', '/Users/Raffaello/iit/codyco-superbuild/build/install/share/codyco');
localName = 'simulink_joint_friction';
setenv('YARP_ROBOT_NAME', joint.robot);

robotName = 'icub';
Ts = 0.01;
name = 'idle';

%% Load from file measure of friction
joint = joint.loadIdleMeasure('idle',10);

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