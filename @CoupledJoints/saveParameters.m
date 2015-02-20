function saveParameters(coupled, file)
    if ~exist('file','var')
        file = 'parameters';
    end
    for i=1:size(coupled.joint,2)
        coupled.joint(i).saveParameters(file);
    end
end