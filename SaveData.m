if exist('tout','var')
    time = tout;
    clear tout;
end
path = robot.path;
q = logsout.get('q').Values.Data;
qD = logsout.get('qD').Values.Data;
qDD = logsout.get('qDD').Values.Data;
tau = logsout.get('tau').Values.Data;

PWM = struct;
PWM.torso = logsout.get('torso').Values.Data;
PWM.left_leg = logsout.get('left_leg').Values.Data;
PWM.right_leg = logsout.get('right_leg').Values.Data;
PWM.left_arm = logsout.get('left_arm').Values.Data;
PWM.right_arm = logsout.get('right_arm').Values.Data;
%Current = struct;
%Current.torso = logsout.get('current').Values.Data;
%Current.left_leg = logsout.get('left_leg').Values.Data;
%Current.right_leg = logsout.get('right_leg').Values.Data;
if exist('Current','var')
    save([path name '.mat'],'q','qD','qDD','tau','PWM','Current','time');
else
    save([path name '.mat'],'q','qD','qDD','tau','PWM','time');
end

disp(['SAVED ON ' path name '.mat']);

clear q qD qDD tau PWM Current;
clear path;