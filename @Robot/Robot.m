classdef Robot
    %ROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        SIMULATOR = 'icubGazeboSim';
        
        codyco_folder;
        build_folder = 'build';
        formatOut = 'yyyymmdd-HH:MM';
        worldRefFrame = 'root_link';
        robot_fixed = 'true';
        configFile = 'yarpWholeBodyInterface_friction.ini';
        start_path;
        joints_avaiable;
    end
    
    properties
        Ts = 0.01;
        realNameRobot;
        joints;
        robotName = 'icub';
        localName = 'simulink_joint_friction';
    end
    
    methods
        function robot = Robot(realNameRobot, start_path, codyco_folder)
            % Name robot
            robot.realNameRobot = realNameRobot;
            % Start path
            robot.start_path = start_path;
            % Codyco folder
            robot.codyco_folder = codyco_folder;
            % folder where exist robot folder
            copy_yarp_file = fullfile(codyco_folder,'libraries','yarpWholeBodyInterface','app','robots',robot.realNameRobot,'yarpWholeBodyInterface.ini');
            
            robot.joints_avaiable = struct;
            % Parse file and build robot
            text = robot.parseFile(copy_yarp_file, 'WBI_YARP_JOINTS');
            for i=1:size(text,2)
                [~,tok] = regexp(text{i}, '(\w+).*? = \((\w+).*?,(\w+).*?\)', 'match','tokens');
                list = tok{:};
                if ~isfield(robot.joints_avaiable,list{2})
                    robot.joints_avaiable.(list{2}) = Joint(list{1},list{2},list{3});
                else
                    robot.joints_avaiable.(list{2}) = [robot.joints_avaiable.(list{2}) Joint(list{1},list{2},list{3})];
                end
            end
            % Load all parameters from file
            path_project = fullfile(robot.start_path,robot.realNameRobot,'JointNameList.ini');
            if exist(path_project,'file')
                robot = robot.loadParameters(path_project);
                disp('[INFO] Load configuration');
            end
        end
        
        function robot = loadParameters(robot, path_project)
            %% Load information about motors
            text = robot.parseFile(path_project, 'JOINT_LIST PARAMETERS');
            for i=1:size(text,2)
                [~,tok] = regexp(text{i}, '(\w+).*? = \((\w+).*?,(\w+).*?,(\w+).*?\)', 'match','tokens');
                list = tok{:};
                part = robot.getPartFromName(list{1});
                if isfield(robot.joints_avaiable,part)
                    joint_part = robot.joints_avaiable.(part);
                    for count=1:size(joint_part,2)
                        if strcmp(list{1},joint_part(count).name)
                            %disp(joint_part(count).name);
                            joint_part(count) = joint_part(count).setMotor(list{2}, list{3}, list{4});
                            robot.joints_avaiable.(part) = joint_part;
                            break;
                        end
                    end
                end
            end
        end
        
        function robot = loadData(robot)
            %% Load all data from path folder
            for i=1:size(robot.joints_avaiable,2)
                path = fullfile(robot.start_path,robot.realNameRobot,robot.joints{i}.part,robot.joints{i}.name);
            end
        end
        
        function buildFolders(robot)
            %% Build folder with all dump and images joints_avaiable
            if size(robot.joints,2) > 0
                for i=1:size(robot.joints,2)
                    if isa(robot.joints{i},'Joint')
                        path = fullfile(robot.start_path,robot.realNameRobot,robot.joints{i}.part,robot.joints{i}.name);
                    else
                        coupled = robot.joints{i};
                        if strcmp(coupled{i}.part,'torso')
                            path = fullfile(robot.start_path,robot.realNameRobot,coupled{i}.part);
                        else
                            path = fullfile(robot.start_path,robot.realNameRobot,coupled{i}.part,'shoulder');
                        end
                    end
                    if ~exist(path,'dir') % Build folder
                        mkdir(path);
                    end
                end
            end
        end
        
        function configure(robot, name, check_yarp)
            %% Configure Yarp Whole Body Interface
            if size(robot.joints,2) > 0
                list = robot.getListJoints(robot.joints{1});
                for i=2:size(robot.joints,2)
                    list = [list ', ' robot.getListJoints(robot.joints{i})];
                end
                if size(list,2) > 0
                    format_list = [name ' = ( ' list ' )'];
                else
                    format_list = '';
                end
                % Plot list of joints_avaiable
                disp(format_list);
                % Set Yarp WBI
                robot.loadYarpWBI(name, format_list);
                % Set WBI
                text = robot.setupWBI(name);
                
                disp('[INFO] Update!');
            else
                
                disp('[ERROR] List without joints_avaiable!');
            end
            % Assignin variables
            assignin('base', 'ROBOT_DOF', size(robot.joints,2));
            assignin('base', 'Ts', robot.Ts);
            %assignin('base', 'nameRobot', robot.robotName);
            %assignin('base', 'localName', robot.localName);
            %setenv('YARP_DATA_DIRS', [codyco_folder '/' build_folder '/install/share/codyco']);
            setenv('YARP_ROBOT_NAME', robot.realNameRobot);
            
            % Set yarp namespace
            if exist('check_yarp','var')
                nameSpace = [ '/' robot.robotName robot.realNameRobot(end-1:end)];
                [~,name_set] = system('yarp namespace');
                if strcmp(check_yarp,'true')
                    if strcmp(name_set(17:end-1), nameSpace) == 0
                        [~,namespace] = system(['yarp namespace ' nameSpace]);
                        text = [text namespace];
                        [~,detect] = system('yarp detect --write');
                        text = [text detect];
                    end
                end
                text = [text sprintf('\n---------\n') name_set];
            end
            disp(text);
        end
        
        function joint = getJoint(robot, name)
            %% Get joint from list
            part = robot.getPartFromName(name);
            if size(part,1) > 0
                group = robot.joints_avaiable.(part);
                for i=1:size(group,2)
                    if strcmp(group(i).name,name)
                        joint{1} = group(i);
                        return
                    end
                end
            end
            joint = {1};
        end
        
        function joints_avaiable = getCoupledJoints(robot,name)
            %% Get Coupled joints_avaiable from list
            if strcmp(name,'torso')
                j = robot.getJoint('torso_yaw');
                joint{1} = j{1};
                j = robot.getJoint('torso_roll');
                joint{2} = j{1};
                j = robot.getJoint('torso_pitch');
                joint{3} = j{1};
                joints_avaiable{1} = joint;
            else
                [~,tok] = regexp(name, '(\w+).*?_(\w+).*?', 'match','tokens');
                if size(tok,1) > 0
                    list = tok{:};
                    if size(list,2) == 2
                        if strcmp(list{2},'shoulder')
                            j = robot.getJoint([name '_pitch']);
                            joint{1} = j{1};
                            j = robot.getJoint([name '_roll']);
                            joint{2} = j{1};
                            j = robot.getJoint([name '_yaw']);
                            joint{3} = j{1};
                            joints_avaiable{1} = joint;
                            return
                        end
                    end
                end
                joints_avaiable = {};
            end
        end
        
        function robot = setReferenceFrame(robot, worldRefFrame, robot_fixed)
            %% Set Reference_frame
            robot.worldRefFrame = worldRefFrame;
            robot.robot_fixed = robot_fixed;
        end
    end
    
    methods(Access = protected)
        function text = setupWBI(robot, name)
            %% Setup Whole Body Interface
            path = fullfile(getenv('HOME'),'.local','share','yarp','contexts','wholeBodyInterfaceToolbox','wholeBodyInterfaceToolbox.ini');
            if exist(path,'file')
                name_yarp_file = cd(path);
                disp('UPDATE LOCAL FOLDER!');
            else
                %name_yarp_file = [build_folder '/main/WBIToolbox/share/codyco/contexts/wholeBodyInterfaceToolbox/wholeBodyInterfaceToolbox.ini'];
                name_yarp_file = fullfile(robot.codyco_folder,robot.build_folder,'install','share','codyco','contexts','wholeBodyInterfaceToolbox','wholeBodyInterfaceToolbox.ini');
            end
            if ~strcmp(robot.realNameRobot, robot.SIMULATOR)
                text = sprintf('robot          %s\n',robot.robotName);
            else
                text = sprintf('robot          %s\n',robot.realNameRobot);
            end
            text = [text sprintf('localName      %s\n',robot.localName)];
            text = [text sprintf('worldRefFrame  %s\n',robot.worldRefFrame)];
            text = [text sprintf('robot_fixed    %s\n',robot.robot_fixed)];
            text = [text sprintf('wbi_id_list    %s\n',name)];
            text = [text sprintf('wbi_config_file %s', robot.configFile)];
            fid = fopen(name_yarp_file,'w');
            fprintf(fid, '%s', text);
            fclose(fid);
        end
        
        function loadYarpWBI(robot, name, list)
            %% Load and save in BUILD directory configuraton
            path = fullfile(getenv('HOME'), '.local', 'share', 'yarp', 'robots', robot.realNameRobot, robot.configFile);
            if exist(path,'file')
                name_yarp_file = cd(path);
                disp('UPDATE LOCAL FOLDER!');
            else
                name_yarp_file = fullfile(robot.codyco_folder,robot.build_folder,'install','share','codyco','robots',robot.realNameRobot,robot.configFile);
            end
            copy_yarp_file = fullfile(robot.codyco_folder,'libraries','yarpWholeBodyInterface','app','robots',robot.realNameRobot,'yarpWholeBodyInterface.ini');
            copyfile(copy_yarp_file, name_yarp_file);
            
            fid = fopen(name_yarp_file, 'a+');
            fprintf(fid, '# TEST JOINT\n%s', [name ' = (' list ')']);
            fclose(fid);
        end
    end
    
    methods (Access = protected, Static)
        function T = getTransformMatrix(part)
            %% Get T matrix from part
            if strcmp(part, 'torso')
                R = 0.04;
                r = 0.022;
                T = [r/R r/(2*R) r/(2*R);
                    0    1/2     1/2;
                    0   -1/2     1/2];
            elseif strcmp(part, 'l_shoulder')
                t = 0.625;
                T = [-1     0	0;
                     -1    -t	0;
                     0      t  -t];
            elseif strcmp(part, 'r_shoulder')
                t = 0.625;
                T = [1      0   0;
                     1      t   0;
                     0     -t   t];
            else
                T = eye(3);
            end
        end
        function list = getListJoints(list_joint)
            %% Get a list of joints_avaiable to add in yarpWholeBodyInterface
            if ~isa(list_joint,'Joint')
                if size(list_joint,2) > 0
                    list = list_joint{1}.name;
                    for i=2:size(list_joint,2)
                        list = [list ', ' list_joint{i}.name];
                    end
                else
                    list = '';
                end
            else
                list = list_joint.name;
            end
        end
        
        function part = getPartFromName(name)
            %% Reconstruct part from name joint
            groups = textscan(lower(name),'%s','delimiter','_');
            groups = groups{1};
            if strcmp(groups{1},'l')
                part = 'left';
            elseif strcmp(groups{1},'r')
                part = 'right';
            else
                if strcmp(groups{1},'torso') || strcmp(groups{1},'head')
                    part = groups{1};
                    return
                else
                    part = '';
                    return;
                end
            end
            if strcmp(groups{2},'shoulder') || strcmp(groups{2},'elbow')
                part = [part '_arm'];
            elseif strcmp(groups{2},'hip') || strcmp(groups{2},'knee') || strcmp(groups{2},'ankle')
                part = [part '_leg'];
            else
                part = '';
                return
            end
        end
        
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
    end
         

%         
%         function name = setupExperiment(robot, type, logsout, time)
%             name = [type '-' datestr(now,robot.formatOut)];
%             number = 0;
%             for i=1:size(robot.joints_avaiable,2)
%                 m = matfile([robot.joints_avaiable{i}.path name '.mat'],'Writable',true);
%                 if robot.isWBIFrictionJoint()
%                     if isa(robot.joints_avaiable{i},'CoupledJoints')
%                         number = (1:size(robot.joints_avaiable{i}.joint,2));
%                         number = number + (i-1);
%                     else
%                         number = number(end) + 1;
%                     end
%                 else
%                     number = robot.joints_avaiable{i}.number;
%                 end
%                 m.time = time;
%                 m.q = logsout.get('q').Values.Data(:,number);
%                 m.qD = logsout.get('qD').Values.Data(:,number);
%                 m.qDD = logsout.get('qDD').Values.Data(:,number);
%                 m.tau = logsout.get('tau').Values.Data(:,number);
%                 PWM = struct;
%                 PWM.(robot.joints_avaiable{i}.group_select) = logsout.get(['pwm_' robot.joints_avaiable{i}.group_select]).Values.Data;
%                 m.PWM = PWM;
%                 Current = struct;
%                 Current.(robot.joints_avaiable{i}.group_select) = logsout.get(['current_' robot.joints_avaiable{i}.group_select]).Values.Data;
%                 m.Current = Current;
%             end
%         end

%         function [robot, counter] = plotAndPrintAllData(robot, name, counter)
%             if ~strcmp(name(1:3),'ref')
%                 type_idle = 1;
%             else
%                 type_idle = 0;  
%             end
%             if ~exist('counter','var')
%                 counter = 1;
%             end
%             for i=1:size(robot.joints_avaiable,2)
%                 if type_idle == 1
%                     robot.joints_avaiable{i} = robot.joints_avaiable{i}.loadIdleMeasure(name);
%                     [ ~, counter] = robot.joints_avaiable{i}.savePictureFriction(counter);
%                 else
%                     robot.joints_avaiable{i} = robot.joints_avaiable{i}.loadRefMeasure(name);
%                     % FIGURE - PWM vs Torque
%                     [ ~, counter] = robot.joints_avaiable{i}.savePictureKt(counter);
%                     robot.joints_avaiable{i}.saveControlToFile();
%                 end
%                 % Save information to file
%                 robot.joints_avaiable{i}.saveToFile();
%             end
%         end
        
%         function robot = addParts(robot, part, type)
%             if strcmp(part,'leg')
%                 robot = robot.addMotor(part, type,'hip','pitch');
%                 robot = robot.addMotor(part, type,'hip','roll');
%                 robot = robot.addMotor(part, type,'hip','yaw');
%                 robot = robot.addMotor(part, type,'knee');
%                 robot = robot.addMotor(part, type,'ankle','roll');
%                 robot = robot.addMotor(part, type,'ankle','yaw');
% %             elseif strcmp(part,'arm')
% %                 robot = robot.addMotor(part, type,'pitch');
% %                 robot = robot.addMotor(part, type,'roll');
% %                 robot = robot.addMotor(part, type,'yaw');
% %                 robot = robot.addMotor(part, type,'elbow');
% %             elseif strcmp(part,'torso')
% %                 
%             end
%         end
        
%         function command = getControlBoardCommand(robot, rate)
%             %% Get string to start ControlBoardDumper
%             if ~exist('rate','var')
%                 rate = 10;
%             end
%             command = ['controlBoardDumper'...
%                 ' --robot icub'...
% %                 ' --part ' joint.group_select ...
%                 ' --rate ' num2str(rate) ...
% %                 ' --joints_avaiable "(' num2str(joint.number_part-1) ')"' ...
%                 ' --dataToDump "(getOutputs getCurrents)"'];
%         end
        
    
end

