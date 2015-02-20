function coupled = loadRefMeasure(coupled, file)
    if ~exist('file','var')
        file = 'ref';
    end
    data = load([coupled.path file '.mat']);

    data.q     = (coupled.T^-1*data.q')';
    data.qD    = (coupled.T^-1*data.qD')';
    data.qDD    = (coupled.T^-1*data.qDD')';
    data.tau = (coupled.T'*data.tau')';

    for i=1:size(coupled.joint,2)
        temp = struct;
        temp.q = data.q(:,i);
        temp.qD = data.qD(:,i);
        temp.qDD = data.qDD(:,i);
        temp.tau = data.tau(:,i);
        temp.PWM = data.PWM;
        temp.Current = data.Current;
        temp.time = data.time;
        coupled.joint(i) = coupled.joint(i).loadReference(temp);
    end
end