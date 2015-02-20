function coupled = loadParameters(coupled, file)
    if ~exist('file','var')
        file = 'parameters';
    end
    for i=1:size(coupled.joint,2)
        coupled.joint(i) = coupled.joint(i).loadParameters(file);
    end
end