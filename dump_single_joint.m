%% Example to use this scripts
% This file is usefull to understand the functionality about all object
% included on this folder

%% Load object Motor
% Set all information about motor and robot when you would like to
% experiments
joint = Motor('experiments','iCubGenova01','leg','left','hip','roll');
% List to load in ---
disp(joint.getWBIlist());
% Start Control board dumper
%disp(joint.getControlBoardCommand());

setenv('YARP_DATA_DIRS', '/Users/Raffaello/iit/codyco-superbuild/build/install/share/codyco');
localName = 'simulink_joint_friction';
setenv('YARP_ROBOT_NAME', joint.robot);

robotName = 'icub';
Ts = 0.01;

%% Save in file
name = 'ref';
path = joint.path;
SaveData;
clear name path;