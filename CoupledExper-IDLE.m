
%% Load object Motor
% Set all information about motor and robot when you would like to
% experiments


joint = CoupledJoints('experiments', 'iCubGenova03', 'torso');
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

%% Save on file
if exist('tout','var')
    time = tout;
    clear tout;
end
save([joint.path name '.mat'],'logsout','time');
disp(['SAVED ON ' joint.path name '.mat']);

