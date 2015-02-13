if exist('tout','var')
    time = tout;
    clear tout;
end

for i=1:size(robot.size,2)
    q = logsout.get('q').Values.Data(:,i);
    qD = logsout.get('qD').Values.Data(:,i);
    qDD = logsout.get('qDD').Values.Data(:,i);
    tau = logsout.get('tau').Values.Data(:,i);
    PWM.(robot.joints(i).group_select) = logsout.get(['pwm_' robot.joints(i).group_select]).Values.Data;
    %Current.(robot.joints(i).group_select) = logsout.get(['current_' robot.joints(i).group_select]).Values.Data;
    %save([robot.joints(i).path name '.mat'],'q','qD','qDD','tau','PWM','Current','time');
    save([robot.joints(i).path name '.mat'],'q','qD','qDD','tau','PWM','time');
    disp(['SAVED ON ' robot.joints(i).path name '.mat']);
    clear q qD qDD tau PWM Current;
end

clear name path;