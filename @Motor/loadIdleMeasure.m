function joint = loadIdleMeasure(joint, file, cutoff)
    %% Load data from mat file
    if ~exist('file','var')
        file = 'idle';
    end
    data = load([joint.path file '.mat']);
    if exist('cutoff','var')
        joint.friction = Friction(data.q, data.qD, data.qDD, data.tau, data.time, cutoff);
        joint.friction = joint.friction.setExperiment(file);
    else
        joint.friction = Friction(data.q, data.qD, data.qDD, data.tau, data.time);
        joint.friction = joint.friction.setExperiment(file);
    end
end
