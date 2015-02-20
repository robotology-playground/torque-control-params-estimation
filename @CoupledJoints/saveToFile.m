function saveToFile(coupled, name)
    %% Save on file
    if ~exist('name','var')
        name = 'data';
    end
    for i=1:size(coupled.joint,2)
        coupled.joint(i).saveToFile(name);
    end
end