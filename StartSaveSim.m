function StartSaveSim( joint, name_file, Reference, Time )
%STARTSIMULATION Summary of this function goes here
%   Detailed explanation goes here

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

end

