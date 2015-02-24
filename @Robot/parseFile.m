function list = parseFile(copy_yarp_file, name_group)
    %% Parse file and get list of lines
    fid = fopen(copy_yarp_file,'r');  % Open text file
    while (~feof(fid))                                     % For each block:
        InputText = textscan(fid,'%s',1,'delimiter','\n');
        
        counter = 1;
        [mat,~] = regexp(InputText{1}, '\[(\w+).*?\]', 'match');
        if size(mat,1) ~= 0
            if size(mat{:},1) ~= 0
                group_file = mat{1};
                if strcmp(group_file,['[' name_group ']'])
                    while (~feof(fid))
                        InputText = textscan(fid,'%s',1,'delimiter','\n');
                        [mat,~] = regexp(InputText{1}, '\[(\w+).*?\]', 'match');
                        if size(mat{:}) ~= 0
                            break;
                        else
                            string = InputText{1};
                            str = ['' string{1}];
                            if size(str,1) > 0
                                if ~strcmp(str(1), '#')
                                    list{counter} = str;
                                    counter = counter + 1;
                                end
                            end
                        end

                    end
                end
            end
        end
    end
    
    fclose(fid);
end