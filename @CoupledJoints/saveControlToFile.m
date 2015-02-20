function saveControlToFile(coupled, name)
    %% Save on file
    if ~exist('name','var')
        name = 'control';
    end
    for i=1:size(coupled.joint,2)
        coupled.joint(i).saveControlToFile(name);
    end
end