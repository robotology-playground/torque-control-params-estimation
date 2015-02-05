%% Example to use this scripts
% This file is usefull to understand the functionality about all object
% included on this folder

%% Load object Motor
% Set all information about motor and robot when you would like to
% experiments
experiment = ExperCollector('iCubGenova01','');
% Load joints
experiment = experiment.addMotor('leg','left','hip','roll');
experiment = experiment.addMotor('leg','right','hip','roll');
experiment = experiment.addMotor('leg','left','ankle','roll');
experiment = experiment.addMotor('leg','right','ankle','roll');
% List to load in ---
disp(experiment.getWBIlist());

setenv('YARP_DATA_DIRS', '/Users/Raffaello/iit/codyco-superbuild/build/install/share/codyco');
localName = 'simulink_joint_friction';
setenv('YARP_ROBOT_NAME', experiment.robot);

robotName = 'icub';
Ts = 0.01;

%% Save in file
name = 'idle';
path = experiment.path;
SaveData;