if exist('tout','var')
    time = tout;
    clear tout;
end

for i=1:size(robot.joints,2)
    if isWBIFrictionJoint(robot)
        number = i;
    else
        number = robot.joints(i).number;
    end
    q = logsout.get('q').Values.Data(:,number);
    qD = logsout.get('qD').Values.Data(:,number);
    qDD = logsout.get('qDD').Values.Data(:,number);
    tau = logsout.get('tau').Values.Data(:,number);
    if strcmp(experiment_type,'FrictionIdentificationImproved')
        Mq = logsout.get('Mq').Values.Data;
        hqdq = logsout.get('hqdq').Values.Data;
        g = logsout.get('g').Values.Data;
        fext = logsout.get('fext').Values.Data;
    end
    PWM.(robot.joints(i).group_select) = logsout.get(['pwm_' robot.joints(i).group_select]).Values.Data;
    Current.(robot.joints(i).group_select) = logsout.get(['current_' robot.joints(i).group_select]).Values.Data;
    if strcmp(experiment_type,'FrictionIdentificationImproved')
        save([robot.joints(i).path name '.mat'],'q','qD','qDD','tau','PWM','Current','time','Mq','hqdq','g','fext');
    else
        save([robot.joints(i).path name '.mat'],'q','qD','qDD','tau','PWM','Current','time');
    end
    disp(['SAVED ON ' robot.joints(i).path name '.mat']);
    clear q qD qDD tau PWM Current Mq hqdq g fext;
end