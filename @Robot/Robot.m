classdef Robot
    %ROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
%         JOINT_FRICTION = 'JOINT_FRICTION';
        SIMULATOR = 'icubGazeboSim';
        
%         nameSpace;
        codyco_folder;
        build_folder = 'build';
        formatOut = 'yyyymmdd-HH:MM';
        worldRefFrame = 'root_link';
        robot_fixed = 'true';
        configFile = 'yarpWholeBodyInterface_friction.ini';
%         
%         ROBOT_DOF = 0;
%         counter_joints = 0;
    end
    
    properties
%         path;
        Ts = 0.01;
        realNameRobot;
        joints;
        robotName = 'icub';
        localName = 'simulink_joint_friction';
    end
    
    methods
        function robot = Robot(realNameRobot, codyco_folder)
            % Name robot
            robot.realNameRobot = realNameRobot;
            % Codyco folder
            robot.codyco_folder = codyco_folder;
            % folder where exist robot folder
            copy_yarp_file = [codyco_folder '/libraries/yarpWholeBodyInterface/app/robots/' robot.realNameRobot '/yarpWholeBodyInterface.ini'];
            
            robot.joints = struct;
            % Parse file and build robot
            text = parseFile(robot, copy_yarp_file, 'WBI_YARP_JOINTS');
            for i=1:size(text,2)
                [~,tok] = regexp(text{i}, '(\w+).*? = \((\w+).*?,(\w+).*?\)', 'match','tokens');
                list = tok{:};
                if ~isfield(robot.joints,list{2})
                    robot.joints.(list{2}) = Joint(list{1},list{2},list{3});
                else
                    robot.joints.(list{2}) = [robot.joints.(list{2}) Joint(list{1},list{2},list{3})];
                end
            end
        end
        
        function buildFolders(robot, path_experiment, list_joint)
            %% Build folder with all dump and images joints
            if size(list_joint,2) > 0
                for i=1:size(list_joint,2)
                    if isa(list_joint{i},'Joint')
                        path = fullfile(path_experiment,robot.realNameRobot,list_joint{i}.part,list_joint{i}.name);
                    else
                        coupled = list_joint{i};
                        if strcmp(coupled{i}.part,'torso')
                            path = fullfile(path_experiment,robot.realNameRobot,coupled{i}.part);
                        else
                            path = fullfile(path_experiment,robot.realNameRobot,coupled{i}.part,'shoulder');
                        end
                    end
                    if ~exist(path,'dir') % Build folder
                        mkdir(path);
                    end
                end
            end
        end
        
        function configure(robot, name, list_joint)
            %% Configure Yarp Whole Body Interface
            if exist('list_joint','var')
                if size(list_joint,2) > 0
                    list = robot.getListJoints(list_joint{1});
                    for i=2:size(list_joint,2)
                        list = [list ', ' robot.getListJoints(list_joint{i})];
                    end
                    if size(list,2) > 0
                        format_list = [name ' = ( ' list ' )'];
                    else
                        format_list = '';
                    end
                    robot.loadYarpWBI(name, format_list);
                    robot.setupWBI(name);
                    disp('[INFO] Update!');
                else
                    disp('[ERROR] List without joints!');
                end
            else
                
            end
        end
        
        function joint = getJoint(robot,name)
            %% Get joint from list
            part = robot.getPartFromName(name);
            if size(part,1) > 0
                group = robot.joints.(part);
                for i=1:size(group,2)
                    if strcmp(group(i).name,name)
                        joint{1} = group(i);
                        return
                    end
                end
            end
            joint = {1};
        end
        
        function joints = getCoupledJoints(robot,name)
            %% Get Coupled joints from list
            if strcmp(name,'torso')
                j = robot.getJoint('torso_yaw');
                joint{1} = j{1};
                j = robot.getJoint('torso_roll');
                joint{2} = j{1};
                j = robot.getJoint('torso_pitch');
                joint{3} = j{1};
                joints{1} = joint;
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
                            joints{1} = joint;
                            return
                        end
                    end
                end
                joints = {};
            end
            
        end
    end
    
    methods(Access = protected)
        function text = setupWBI(robot, name)
            %% Setup Whole Body Interface
            if exist([getenv('HOME') '/.local/share/yarp/contexts/wholeBodyInterfaceToolbox/wholeBodyInterfaceToolbox.ini'],'file')
                name_yarp_file = cd([getenv('HOME') '/.local/share/yarp/contexts/wholeBodyInterfaceToolbox/wholeBodyInterfaceToolbox.ini']);
                disp('UPDATE LOCAL FOLDER!');
            else
                %name_yarp_file = [build_folder '/main/WBIToolbox/share/codyco/contexts/wholeBodyInterfaceToolbox/wholeBodyInterfaceToolbox.ini'];
                name_yarp_file = [robot.codyco_folder '/' robot.build_folder '/install/share/codyco/contexts/wholeBodyInterfaceToolbox/wholeBodyInterfaceToolbox.ini'];
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
            if exist([getenv('HOME') '/.local/share/yarp/robots/' robot.realNameRobot '/' robot.configFile],'file')
                name_yarp_file = cd([getenv('HOME') '/.local/share/yarp/robots/' robot.realNameRobot '/' robot.configFile]);
                disp('UPDATE LOCAL FOLDER!');
            else
                name_yarp_file = [robot.codyco_folder '/' robot.build_folder '/install/share/codyco/robots/' robot.realNameRobot '/' robot.configFile];
            end
            copy_yarp_file = [robot.codyco_folder '/libraries/yarpWholeBodyInterface/app/robots/' robot.realNameRobot '/yarpWholeBodyInterface.ini'];
            copyfile(copy_yarp_file, name_yarp_file);
            
            fid = fopen(name_yarp_file, 'a+');
            fprintf(fid, '# TEST JOINT\n%s', [name ' = (' list ')']);
            fclose(fid);
        end
    end
    
    methods (Access = protected, Static)
        function list = getListJoints(list_joint)
            %% Get a list of joints to add in yarpWholeBodyInterface
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
    end
%         function robot = setNameSpace(nameSpace)
%             %% Configure type of namespace
%             robot.nameSpace = nameSpace;
%         end
%         
%         function robot = setNameList(robot, NAME_LIST)
%             if ~strcmp(robot.WBI_LIST, NAME_LIST)
%                 robot.ROBOT_DOF = 25;
%             end
%             robot.WBI_LIST = NAME_LIST;
%         end
%         
%         function list = getJointList(robot)
%             list = robot.joint_list;
%         end
%             
%         function robot = setConfiguration(robot, worldRefFrame, robot_fixed, configFile)
%             robot.worldRefFrame = worldRefFrame;
%             robot.robot_fixed = robot_fixed;
%             if exist('configFile','var')
%                 robot.configFile = configFile;
%             end
%         end
%         
%         function name = setupExperiment(robot, type, logsout, time)
%             name = [type '-' datestr(now,robot.formatOut)];
%             number = 0;
%             for i=1:size(robot.joints,2)
%                 m = matfile([robot.joints{i}.path name '.mat'],'Writable',true);
%                 if robot.isWBIFrictionJoint()
%                     if isa(robot.joints{i},'CoupledJoints')
%                         number = (1:size(robot.joints{i}.joint,2));
%                         number = number + (i-1);
%                     else
%                         number = number(end) + 1;
%                     end
%                 else
%                     number = robot.joints{i}.number;
%                 end
%                 m.time = time;
%                 m.q = logsout.get('q').Values.Data(:,number);
%                 m.qD = logsout.get('qD').Values.Data(:,number);
%                 m.qDD = logsout.get('qDD').Values.Data(:,number);
%                 m.tau = logsout.get('tau').Values.Data(:,number);
%                 PWM = struct;
%                 PWM.(robot.joints{i}.group_select) = logsout.get(['pwm_' robot.joints{i}.group_select]).Values.Data;
%                 m.PWM = PWM;
%                 Current = struct;
%                 Current.(robot.joints{i}.group_select) = logsout.get(['current_' robot.joints{i}.group_select]).Values.Data;
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
%             for i=1:size(robot.joints,2)
%                 if type_idle == 1
%                     robot.joints{i} = robot.joints{i}.loadIdleMeasure(name);
%                     [ ~, counter] = robot.joints{i}.savePictureFriction(counter);
%                 else
%                     robot.joints{i} = robot.joints{i}.loadRefMeasure(name);
%                     % FIGURE - PWM vs Torque
%                     [ ~, counter] = robot.joints{i}.savePictureKt(counter);
%                     robot.joints{i}.saveControlToFile();
%                 end
%                 % Save information to file
%                 robot.joints{i}.saveToFile();
%             end
%         end
%         
%         function bool = isWBIFrictionJoint(robot)
%             bool = strcmp(robot.WBI_LIST, robot.JOINT_FRICTION);
%         end
%         
%         
%         function robot = addMotor(robot, part, type, info1, info2)
%             %% Add motor in robot
%             if exist('type','var') && exist('info1','var') && exist('info2','var')
%                 motor = Motor(robot.path_experiment, robot.nameThisRobot, part, type, info1, info2);
%             elseif exist('type','var') && exist('info1','var') && ~exist('info2','var')
%                 motor = Motor(robot.path_experiment, robot.nameThisRobot, part, type, info1);
%             elseif exist('type','var') && ~exist('info1','var') && ~exist('info2','var')
%                 if strcmp(part,'arm')
%                     motor = CoupledJoints(robot.path_experiment, robot.nameThisRobot, part, type);
%                 else
%                     motor = Motor(robot.path_experiment, robot.nameThisRobot, part, type);
%                 end
%             elseif ~exist('type','var') && ~exist('info1','var') && ~exist('info2','var')
%                 if strcmp(part,'torso')
%                     motor = CoupledJoints(robot.path_experiment, robot.nameThisRobot, part);
%                 else
%                     motor = Motor(robot.path_experiment, robot.nameThisRobot, part);
%                 end
%                 
%             end
%             if isprop(motor,'joint')
%                 robot.ROBOT_DOF = robot.ROBOT_DOF + size(motor.joint,2);
%             else
%                 robot.ROBOT_DOF = robot.ROBOT_DOF + 1;
%             end
%             
%             if isWBIFrictionJoint(robot)
%                 robot.joints{size(robot.joints,2)+1} = motor;
%             else
%                 robot.joints{motor.number} = motor;
%             end
%             
%             if strcmp(robot.joint_list,'')
%                 robot.joint_list = motor.getJointList();
%             else
%                 robot.joint_list = [robot.joint_list ', ' motor.getJointList()];
%             end
%             
%             if exist([robot.joints{end}.path 'parameters.mat'],'file')
%                 robot.joints{end} = robot.joints{end}.loadParameters('parameters');
%             end
%         end
        
%         function robot = setInLastRatio(robot, Voltage, range_pwm)
%             if exist('Voltage','var') && exist('range_pwm','var')
%                 robot.joints{end} = robot.joints{end}.setRatio(Voltage, range_pwm);
%             else
%                 robot.joints{end} = robot.joints{end}.loadParameters(name_parameters);
%             end
%         end
        
%         function saveInLastParameters(robot)
%             robot.joints{end}.saveParameters();
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
% %                 ' --joints "(' num2str(joint.number_part-1) ')"' ...
%                 ' --dataToDump "(getOutputs getCurrents)"'];
%         end
        
%         function configure(robot, check_yarp, codyco_folder, build_folder)
%             %% Configure PC end set YARP NAMESPACE
%             if size(robot.joint_list,2) ~= 0
%             if ~exist('build_folder','var')
%                 build_folder = 'build';
%             end
%             text = '';
%             
%             [~,name_set] = system('yarp namespace');
%             if strcmp(check_yarp,'true')
%                 if strcmp(name_set(17:end-1),robot.nameSpace) == 0
%                     [~,namespace] = system(['yarp namespace ' robot.nameSpace]);
%                     text = [text namespace];
%                     [~,detect] = system('yarp detect --write');
%                     text = [text detect];
%                 end
%             else
%                 text = [text name_set];
%             end
%             % Add configuration WBI
%             text = [text  robot.setupWBI(codyco_folder, build_folder)];
%             % Add JOINT FRICTION in yarpWholeBodyInterface.ini
%             if isWBIFrictionJoint(robot)
%                 robot.loadYarpWBI(codyco_folder, build_folder);
%             end
%             % Set variables environment
%             assignin('base', 'Ts', robot.Ts);
%             %assignin('base', 'nameRobot', robot.robotName);
%             %assignin('base', 'localName', robot.localName);
%             assignin('base', 'ROBOT_DOF', robot.ROBOT_DOF);
%             %setenv('YARP_DATA_DIRS', [codyco_folder '/' build_folder '/install/share/codyco']);
%             setenv('YARP_ROBOT_NAME', robot.nameThisRobot);
%             disp(text);
%             else
%                 disp('You does not load anything motors!');
%             end
%         end
    
end

