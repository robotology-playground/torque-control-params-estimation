function text = saveControlToFile(joint, name)
    %% Save information to txt file
    if size(joint.Kt,1) ~= 0
        if ~exist('name','var')
            name = 'control';
        end
        fileID = fopen([joint.path name '.txt'],'w');
        fprintf(fileID,'%s',joint.textControlData());
        % Close
        fclose(fileID);
    end
end