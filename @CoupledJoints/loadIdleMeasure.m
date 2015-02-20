function coupled = loadIdleMeasure(coupled, file, cutoff)
    if ~exist('file','var')
        file = 'idle';
    end
    coupled.name_experiment = file;
    data = load([coupled.path file '.mat']);
    data.m     = (coupled.T^-1*data.q')';
    data.mD    = (coupled.T^-1*data.qD')';
    data.mDD    = (coupled.T^-1*data.qDD')';
    data.tauM = (coupled.T'*data.tau')';

    for i=1:size(coupled.joint,2)
        if exist('cutoff','var')
            coupled.joint(i).friction = Friction(data.m(:,i), data.mD(:,i), data.mDD(:,i), data.tauM(:,i), data.time, cutoff);
            coupled.joint(i).friction = coupled.joint(i).friction.setExperiment(coupled.name_experiment);
        else
            coupled.joint(i).friction = Friction(data.m(:,i), data.mD(:,i), data.mDD(:,i), data.tauM(:,i), data.time);
            coupled.joint(i).friction = coupled.joint(i).friction.setExperiment(coupled.name_experiment);
        end
    end
end