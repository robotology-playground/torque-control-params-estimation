function joint = loadRefMeasure(joint, file)
    %% Load reference from file
    if ~exist('file','var')
        file = 'ref';
    end
    data = load([joint.path file '.mat']);
    joint = joint.loadReference(data);
end