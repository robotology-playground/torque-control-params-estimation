%% Example to use this scripts
% This file is usefull to understand the functionality about all object
% included on this folder

%% Load object Motor
% Set all information about motor and robot when you would like to
% experiments
joint = Motor('experiments','iCubGenova03','arm','left','hip','pitch');
% Set number of joint you want control and read
joint = joint.setPart('number_joint',1);

%% Set variable environment
setenv('YARP_DATA_DIRS', '/Users/Raffaello/iit/codyco-superbuild/build/install/share/codyco');
localName = 'simulink_joint_friction';
setenv('YARP_ROBOT_NAME', joint.robot);

ROBOT_DOF = 1;
robotName = 'icub';
Ts = 0.01;