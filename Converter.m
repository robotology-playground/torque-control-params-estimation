%% Convert data
% Use this script if you want convert old data for new Friction and Motor
% object

%% Load object Motor
% Set all information about motor and robot when you would like to
% experiments
% experiment = ExperCollector('iCubGenova01');
% % Load joints
% experiment = experiment.addMotor('leg','left','hip','roll');
% experiment = experiment.addMotor('leg','right','hip','roll');
% experiment = experiment.addMotor('leg','left','ankle','roll');
% experiment = experiment.addMotor('leg','right','ankle','roll');
% 
% %% Split in more files
% name = 'idle-2F';
% for i=1:size(experiment.joint,2)
%     data = load([experiment.data_path name '.mat']);
%     time = data.time;
%     q = data.q(:,i);
%     qD = data.qD(:,i);
%     qDD = data.qDD(:,i);
%     tau = data.tau(:,i);
%     PWM = data.PWM;
%     if exist('Current','var')
%         save([experiment.joint(i).path name '.mat'],'q','qD','qDD','tau','PWM','Current','time');
%     else
%         save([experiment.joint(i).path name '.mat'],'q','qD','qDD','tau','PWM','time');
%     end
%     clear q qD qDD tau PWM Current;
% end

robot = 'iCubGenova04';
name = 'idle';

type = 'leg';
part = 'right';
joint = Motor('experiments/OLD',robot,type,part,'knee');
data = load([joint.path name '.mat']);

jointN = Motor('experiments',robot,type,part,'knee');

time = data.time;
q = data.logsout.get('q').Values.Data;%(:,joint.number);
qD = data.logsout.get('qD').Values.Data;%(:,joint.number);
if data.logsout.get('qDD') ~= 0
    qDD = data.logsout.get('qDD').Values.Data;%(:,joint.number);
else
    qDD = [];
end
tau = data.logsout.get('tau').Values.Data;%(:,joint.number);
PWM = struct;
temp_pwm = zeros(size(q,1),5);
temp_pwm(:,joint.number_part) = data.logsout.get('pwm').Values.Data;
PWM.(joint.group_select) = temp_pwm;
if exist('Current','var')
    save([jointN.path name '.mat'],'q','qD','qDD','tau','PWM','Current','time');
else
    save([jointN.path name '.mat'],'q','qD','qDD','tau','PWM','time');
end
clear type_n part_n robot temp_pwm;