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

robot = 'iCubGenova03';
name = 'idle';

type = 'leg';
part = 'right';
joint = Motor('experiments/OLD/20150127',robot,type,part,'knee');
data = load([joint.path name '.mat']);

jointN = Motor('experiments',robot,type,part,'knee');
%%
jointN = CoupledJoints('iCubGenova03','torso');
name = 'ref-11:36';
data = struct;
data.logsout = logsout;
data.time = time;

q = zeros(size(data.logsout.get('q').Values.Data,1),3);
qD = zeros(size(data.logsout.get('q').Values.Data,1),3);
qDD = zeros(size(data.logsout.get('q').Values.Data,1),3);
tau = zeros(size(data.logsout.get('q').Values.Data,1),3);
time = data.time;
for i=3:-1:1
    q(:,4-i) = data.logsout.get('q').Values.Data(:,i);
    qD(:,4-i) = data.logsout.get('qD').Values.Data(:,i);
%     if data.logsout.get('qDD') ~= 0
%         qDD = data.logsout.get('qDD').Values.Data(:,i);
%     else
%         qDD = [];
%     end
    tau(:,4-i) = data.logsout.get('tau').Values.Data(:,i);
end
PWM = struct;
PWM.torso = data.logsout.get('pwm').Values.Data;
Current = struct;
Current.torso = data.logsout.get('current').Values.Data;
% temp_pwm = zeros(size(q,1),5);
% temp_pwm(:,joint.number_part) = data.logsout.get('pwm').Values.Data;
% PWM.(joint.group_select) = temp_pwm;
if exist('Current','var')
    save([jointN.path name '.mat'],'q','qD','qDD','tau','PWM','Current','time');
else
    save([jointN.path name '.mat'],'q','qD','qDD','tau','PWM','time');
end
clear type_n part_n robot temp_pwm;