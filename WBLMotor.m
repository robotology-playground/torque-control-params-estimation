

robotName = 'iCubGenova03';
localName = 'simulink';
Ts = 0.01;

joint = JointStructure(robotName,'leg','left','knee');

if ~exist(joint.folder_path,'dir')
    mkdir(joint.folder_path);
end

name_file = 'ref-1';

%% Reference configuration
Reference = struct;

if strcmp(name_file,'idle')
    Reference.sinAmp = 0;
    Reference.sinFreq = 0;
    Reference.sinBias = 0;
elseif strcmp(name_file,'ref')
    Reference.sinAmp = 10;
    Reference.sinFreq = 0.1;
    Reference.sinBias = 0;
end
Time = 60;

%% Start simulation

disp('Change configuration on JTC and Calibrate (WBD)!');
disp(['NOW IT IS SELECTED: ' joint.folder_path]);
disp(' ');
disp(['Run ' name_file ' simulation!']);
disp('Press any key to continue...');
pause;
disp('RUN...');

sim_data = sim('MotorIdent','ReturnWorkspaceOutputs', 'on');
out = sim_data.get('out');
time = sim_data.get('tout');

%% Salve on file
save([joint.folder_path name_file '.mat'],'out','time');
disp(['SAVED ON ' joint.folder_path name_file '.mat']);


%% Clear
clear Reference robotName localName Ts name_file logsout;