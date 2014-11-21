

joint = Motor('iCubGenova03','leg','left','hip','roll');
Time = 60;

%% 
name = 'idle';

%% Reference configuration
Reference = struct;

if strcmp(name,'idle')
    Reference.sinAmp = 0;
    Reference.sinFreq = 0;
    Reference.sinBias = 0;
elseif strcmp(name,'ref')
    Reference.sinAmp = 10;
    Reference.sinFreq = 0.1;
    Reference.sinBias = 0;
end


%% Start simulation
name_file = [name ''];

localName = 'simulink';
Ts = 0.01;
robotName = joint.robotName;

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

% Save on file
save([joint.folder_path name_file '.mat'],'out','time');
disp(['SAVED ON ' joint.folder_path name_file '.mat']);

%% Clear Data
clear Reference name_file;

