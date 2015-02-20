function saveControlToFile(joint, name)
    %% Save information to txt file
    if size(joint.Kt,1) ~= 0
        if ~exist('name','var')
            name = 'control';
        end
        fileID = fopen([joint.path name '.txt'],'w');
        % Information joint estimation
        fprintf(fileID,'Name: %s\n',joint.robot);
        fprintf(fileID,'Part: %s\n',joint.part);
        if(joint.type ~= 0)
            fprintf(fileID,'Type: %s\n',joint.type);
        end
        if(joint.info1 ~= 0)
            fprintf(fileID,'Info1: %s\n',joint.info1);
        end
        if(joint.info2 ~= 0)
            fprintf(fileID,'Info2: %s\n',joint.info2);
        end
        fprintf(fileID,'\nFriction\n');
        fprintf(fileID,'kc+: %12.8f [V] - kc-: %12.8f [V] \n',joint.friction.KcP/joint.Kt, joint.friction.KcN/joint.Kt);
        fprintf(fileID,'ks+: %12.8f [V][s]/[deg] - kv-: %12.8f [V][s]/[deg]\n',joint.friction.KvP/joint.Kt, joint.friction.KvN/joint.Kt);
        %fprintf(fileID,'KsP: %12.8f [Nm] - KsN %12.8f [Nm][s]/[deg]\n',joint.friction.KsP, joint.friction.KsN);
        if(joint.Kt ~= 0)
            fprintf(fileID,'kt: %12.8f [V]/[Nm]\n',1/joint.Kt);
        end

        % Close
        fclose(fileID);
    end
end