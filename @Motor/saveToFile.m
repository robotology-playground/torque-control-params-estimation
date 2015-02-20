function saveToFile(joint, name)
    %% Save on file
    if ~exist('name','var')
        name = 'data';
    end
    fileID = fopen([joint.path name '.txt'],'w');
    fprintf(fileID,'%s',joint.saveCoeffToFile());
    % Close
    fclose(fileID);
end