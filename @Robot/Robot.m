classdef Robot
    %ROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        JOINT_FRICTION = 'JOINT_FRICTION';
        SIMULATOR = 'icubGazeboSim';

        path_experiment;
        nameSpace;
        formatOut = 'yyyymmdd-HH:MM';
        worldRefFrame = 'root_link';
        robot_fixed = 'true';
        configFile = 'yarpWholeBodyInterface_friction.ini';
        
        ROBOT_DOF = 0;
        counter_joints = 0;
    end
    
    properties
        joint_list = '';
        WBI_LIST;
        localName = 'simulink_joint_friction';
        path;
        Ts = 0.01;
        joints;
        nameThisRobot;
        robotName = 'icub';
    end
    
    methods
        function robot = Robot(nameThisRobot, path_experiment)
            robot.WBI_LIST = robot.JOINT_FRICTION;
            robot.nameThisRobot = nameThisRobot;
            robot.nameSpace = [ '/' robot.robotName nameThisRobot(end-1:end)];
            if ~exist('path_experiment','var')
                robot.path_experiment = 'experiments';
            else
                robot.path_experiment = path_experiment;
            end
            robot.path = [robot.path_experiment '/' robot.nameThisRobot '/'];
        end
        
        function robot = setNameSpace(nameSpace)
            %% Configure type of namespace
            robot.nameSpace = nameSpace;
        end
        
        function robot = setNameList(robot, NAME_LIST)
            if ~strcmp(robot.WBI_LIST, NAME_LIST)
                robot.ROBOT_DOF = 25;
            end
            robot.WBI_LIST = NAME_LIST;
        end
        
        function list = getJointList(robot)
            list = robot.joint_list;
        end
            
        function robot = setConfiguration(robot, worldRefFrame, robot_fixed, configFile)
            robot.worldRefFrame = worldRefFrame;
            robot.robot_fixed = robot_fixed;
            if exist('configFile','var')
                robot.configFile = configFile;
            end
        end
        
        function name = setupExperiment(robot, type, logsout, time)
            name = [type '-' datestr(now,robot.formatOut)];
            number = 0;
            for i=1:size(robot.joints,2)
                m = matfile([robot.joints{i}.path name '.mat'],'Writable',true);
                if robot.isWBIFrictionJoint()
                    if isa(robot.joints{i},'CoupledJoints')
                        number = (1:size(robot.joints{i}.joint,2));
                        number = number + (i-1);
                    else
                        number = number(end) + 1;
                    end
                else
                    number = robot.joints{i}.number;
                end
                m.time = time;
                m.q = logsout.get('q').Values.Data(:,number);
                m.qD = logsout.get('qD').Values.Data(:,number);
                m.qDD = logsout.get('qDD').Values.Data(:,number);
                m.tau = logsout.get('tau').Values.Data(:,number);
                PWM = struct;
                PWM.(robot.joints{i}.group_select) = logsout.get(['pwm_' robot.joints{i}.group_select]).Values.Data;
                m.PWM = PWM;
                Current = struct;
                Current.(robot.joints{i}.group_select) = logsout.get(['current_' robot.joints{i}.group_select]).Values.Data;
                m.Current = Current;
            end
        end
        
        function [robot, counter] = plotAndPrintAllData(robot, name, counter)
            if ~strcmp(name(1:3),'ref')
                type_idle = 1;
            else
                type_idle = 0;  
            end
            if ~exist('counter','var')
                counter = 1;
            end
            for i=1:size(robot.joints,2)
                if type_idle == 1
                    robot.joints{i} = robot.joints{i}.loadIdleMeasure(name);
                    [ ~, counter] = robot.joints{i}.savePictureFriction(counter);
                else
                    robot.joints{i} = robot.joints{i}.loadRefMeasure(name);
                    % FIGURE - PWM vs Torque
                    [ ~, counter] = robot.joints{i}.savePictureKt(counter);
                    robot.joints{i}.saveControlToFile();
                end
                % Save information to file
                robot.joints{i}.saveToFile();
            end
        end
        
        function bool = isWBIFrictionJoint(robot)
            bool = strcmp(robot.WBI_LIST, robot.JOINT_FRICTION);
        end
        
        
        function robot = addMotor(robot, part, type, info1, info2)
            %% Add motor in robot
            if exist('type','var') && exist('info1','var') && exist('info2','var')
                motor = Motor(robot.path_experiment, robot.nameThisRobot, part, type, info1, info2);
            elseif exist('type','var') && exist('info1','var') && ~exist('info2','var')
                motor = Motor(robot.path_experiment, robot.nameThisRobot, part, type, info1);
            elseif exist('type','var') && ~exist('info1','var') && ~exist('info2','var')
                if strcmp(part,'arm')
                    motor = CoupledJoints(robot.path_experiment, robot.nameThisRobot, part, type);
                else
                    motor = Motor(robot.path_experiment, robot.nameThisRobot, part, type);
                end
            elseif ~exist('type','var') && ~exist('info1','var') && ~exist('info2','var')
                if strcmp(part,'torso')
                    motor = CoupledJoints(robot.path_experiment, robot.nameThisRobot, part);
                else
                    motor = Motor(robot.path_experiment, robot.nameThisRobot, part);
                end
                
            end
            if isprop(motor,'joint')
                robot.ROBOT_DOF = robot.ROBOT_DOF + size(motor.joint,2);
            else
                robot.ROBOT_DOF = robot.ROBOT_DOF + 1;
            end
            
            if isWBIFrictionJoint(robot)
                robot.joints{size(robot.joints,2)+1} = motor;
            else
                robot.joints{motor.number} = motor;
            end
            
            if strcmp(robot.joint_list,'')
                robot.joint_list = motor.getJointList();
            else
                robot.joint_list = [robot.joint_list ', ' motor.getJointList()];
            end
            
            if exist([robot.joints{end}.path 'parameters.mat'],'file')
                robot.joints{end} = robot.joints{end}.loadParameters('parameters');
            end
        end
        
        function robot = setInLastRatio(robot, Voltage, range_pwm)
            if exist('Voltage','var') && exist('range_pwm','var')
                robot.joints{end} = robot.joints{end}.setRatio(Voltage, range_pwm);
            else
                robot.joints{end} = robot.joints{end}.loadParameters(name_parameters);
            end
        end
        
        function saveInLastParameters(robot)
            robot.joints{end}.saveParameters();
        end
        
        function robot = addParts(robot, part, type)
            if strcmp(part,'leg')
                robot = robot.addMotor(part, type,'hip','pitch');
                robot = robot.addMotor(part, type,'hip','roll');
                robot = robot.addMotor(part, type,'hip','yaw');
                robot = robot.addMotor(part, type,'knee');
                robot = robot.addMotor(part, type,'ankle','roll');
                robot = robot.addMotor(part, type,'ankle','yaw');
%             elseif strcmp(part,'arm')
%                 robot = robot.addMotor(part, type,'pitch');
%                 robot = robot.addMotor(part, type,'roll');
%                 robot = robot.addMotor(part, type,'yaw');
%                 robot = robot.addMotor(part, type,'elbow');
%             elseif strcmp(part,'torso')
%                 
            end
        end
        
        function command = getControlBoardCommand(robot, rate)
            %% Get string to start ControlBoardDumper
            if ~exist('rate','var')
                rate = 10;
            end
            command = ['controlBoardDumper'...
                ' --robot icub'...
%                 ' --part ' joint.group_select ...
                ' --rate ' num2str(rate) ...
%                 ' --joints "(' num2str(joint.number_part-1) ')"' ...
                ' --dataToDump "(getOutputs getCurrents)"'];
        end
        
        function configure(robot, check_yarp, codyco_folder, build_folder)
            %% Configure PC end set YARP NAMESPACE
            if size(robot.joint_list,2) ~= 0
            if ~exist('build_folder','var')
                build_folder = 'build';
            end
            text = '';
            
            [~,name_set] = system('yarp namespace');
            if strcmp(check_yarp,'true')
                if strcmp(name_set(17:end-1),robot.nameSpace) == 0
                    [~,namespace] = system(['yarp namespace ' robot.nameSpace]);
                    text = [text namespace];
                    [~,detect] = system('yarp detect --write');
                    text = [text detect];
                end
            end
            text = [text name_set];
            % Add configuration WBI
            text = [text  robot.setupWBI(codyco_folder, build_folder)];
            % Add JOINT FRICTION in yarpWholeBodyInterface.ini
            if isWBIFrictionJoint(robot)
                robot.loadYarpWBI(codyco_folder, build_folder);
            end
            % Set variables environment
            assignin('base', 'Ts', robot.Ts);
            %assignin('base', 'nameRobot', robot.robotName);
            %assignin('base', 'localName', robot.localName);
            assignin('base', 'ROBOT_DOF', robot.ROBOT_DOF);
            %setenv('YARP_DATA_DIRS', [codyco_folder '/' build_folder '/install/share/codyco']);
            setenv('YARP_ROBOT_NAME', robot.nameThisRobot);
            disp(text);
            else
                disp('You does not load anything motors!');
            end
        end
    end
    
    methods
        function text = setupWBI(robot, codyco_folder, build_folder)
            %name_yarp_file = [build_folder '/main/WBIToolbox/share/codyco/contexts/wholeBodyInterfaceToolbox/wholeBodyInterfaceToolbox.ini'];
            
            if exist([getenv('HOME') '/.local/share/yarp/contexts/wholeBodyInterfaceToolbox/wholeBodyInterfaceToolbox.ini'],'file')
                name_yarp_file = cd([getenv('HOME') '/.local/share/yarp/contexts/wholeBodyInterfaceToolbox/wholeBodyInterfaceToolbox.ini']);
                disp('UPDATE LOCAL FOLDER!');
            else
                name_yarp_file = [codyco_folder '/' build_folder '/install/share/codyco/contexts/wholeBodyInterfaceToolbox/wholeBodyInterfaceToolbox.ini'];
            end
            if ~strcmp(robot.nameThisRobot, robot.SIMULATOR)
                text = sprintf('robot          %s\n',robot.robotName);
            else
                text = sprintf('robot          %s\n',robot.nameThisRobot);
            end
            text = [text sprintf('localName      %s\n',robot.localName)];
            text = [text sprintf('worldRefFrame  %s\n',robot.worldRefFrame)];
            text = [text sprintf('robot_fixed    %s\n',robot.robot_fixed)];
            text = [text sprintf('wbi_id_list    %s\n',robot.WBI_LIST)];
            text = [text sprintf('wbi_config_file %s', robot.configFile)];
            fid = fopen(name_yarp_file,'w');
            fprintf(fid, '%s', text);
            fclose(fid);
        end
        
        function loadYarpWBI(robot, codyco_folder, build_folder)
            %% Load and save in BUILD directory configuraton
            if exist([getenv('HOME') '/.local/share/yarp/robots/' robot.nameThisRobot '/' robot.configFile],'file')
                name_yarp_file = cd([getenv('HOME') '/.local/share/yarp/robots/' robot.nameThisRobot '/' robot.configFile]);
                disp('UPDATE LOCAL FOLDER!');
            else
                name_yarp_file = [codyco_folder '/' build_folder '/install/share/codyco/robots/' robot.nameThisRobot '/' robot.configFile];
            end
            copy_yarp_file = [codyco_folder '/libraries/yarpWholeBodyInterface/app/robots/' robot.nameThisRobot '/yarpWholeBodyInterface.ini'];
            copyfile(copy_yarp_file, name_yarp_file);
            
            fid = fopen(name_yarp_file, 'a+');
            command = [robot.WBI_LIST ' = (' robot.joint_list ')'];
            fprintf(fid, '# TEST JOINT\n%s', command);
            fclose(fid);
        end
    end
    
end

