

robotName = 'iCubGenova03';
localName = 'simulink';
Tc = 0.01;

joint = JointStructure(robotName,'leg','right','knee');

if ~exist(joint.folder_path,'dir')
    mkdir(joint.folder_path);
end

name_file = 'idle';

%% Reference configuration
Reference = struct;
Reference.sinAmp = 4;
Reference.sinFreq = 0.1;
Reference.sinBias = 0;
Time = 60;

%% Start simulation

disp('Change configuration on JTC and Calibrate!');
disp(['NOW IT IS SELECTED: ' joint.folder_path]);
disp(' ');
disp(['Run ' name_file ' simulation!']);
disp('Press any key to continue...');
pause;
disp('RUN...');

sim_data = sim('MotorIdent','ReturnWorkspaceOutputs', 'on');
out = sim_data.get('out');
time = sim_data.get('tout');

save([joint.folder_path name_file '.mat'],'out','time');
disp(['SAVED ON ' joint.folder_path name_file '.mat']);

%% clear
clear Reference Time robotName localName Tc name_file logsout;