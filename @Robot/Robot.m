classdef Robot
    %ROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        JOINT_FRICTION = 'JOINT_FRICTION';
        robot_type = 'icub';
        path_experiment;
        joint_list = '';
        nameSpace;
        formatOut = 'yyyymmdd-HH:MM';
        worldRefFrame = 'root_link';
        robot_fixed = 'true';
        configFile = 'yarpWholeBodyInterface.ini';
        
        ROBOT_DOF = 0;
    end
    
    properties
        WBI_LIST;
        localName = 'simulink_joint_friction';
        robotName;
        path;
        Ts = 0.01;
        joints;
        coupledjoints;
    end
    
    methods
        function robot = Robot(robotName, path_experiment)
            robot.WBI_LIST = robot.JOINT_FRICTION;
            robot.robotName = robotName;
            robot.nameSpace = [ '/' robot.robot_type robotName(end-1:end)];
            if ~exist('path_experiment','var')
                robot.path_experiment = 'experiments';
            else
                robot.path_experiment = path_experiment;
            end
            robot.path = [robot.path_experiment '/' robot.robotName '/'];
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
        
        function name = setupExperiment(robot, type)
            name = [type '-' datestr(now,robot.formatOut)];
        end
        
        function bool = isWBIFrictionJoint(robot)
            bool = strcmp(robot.WBI_LIST, robot.JOINT_FRICTION);
        end
        
        
        function robot = addMotor(robot, part, type, info1, info2)
            %% Add motor in robot
            if exist('type','var') && exist('info1','var') && exist('info2','var')
                motor = Motor(robot.path_experiment, robot.robotName, part, type, info1, info2);
            elseif exist('type','var') && exist('info1','var') && ~exist('info2','var')
                motor = Motor(robot.path_experiment, robot.robotName, part, type, info1);
            elseif exist('type','var') && ~exist('info1','var') && ~exist('info2','var')
                motor = Motor(robot.path_experiment, robot.robotName, part, type);
            elseif ~exist('type','var') && ~exist('info1','var') && ~exist('info2','var')
                motor = Motor(robot.path_experiment, robot.robotName, part);
            end
            robot.joints = [robot.joints motor];
            
            if strcmp(robot.joint_list,'')
                robot.joint_list = motor.WBIname;
            else
                robot.joint_list = [robot.joint_list ', ' motor.WBIname];
            end
            if isWBIFrictionJoint(robot)
                robot.ROBOT_DOF = size(robot.joints,2);
            end
        end
        
        function robot = addCoupledJoints(robot, part, type)
            if exist('type','var')
                motor = CoupledJoints(robot.path_experiment, robot.robotName, part, type);
            else
                motor = CoupledJoints(robot.path_experiment, robot.robotName, part);
            end
            robot.coupledjoints = [robot.coupledjoints motor];
            
            if strcmp(robot.joint_list,'')
                robot.joint_list = motor.WBIname;
            else
                robot.joint_list = [robot.joint_list ', ' motor.getJointList()];
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
        
        function configure(robot, codyco_folder, build_folder)
            %% Configure PC end set YARP NAMESPACE
            if size(robot.joints,2) ~= 0
            if ~exist('build_folder','var')
                build_folder = 'build';
            end
            text = '';
            
            [~,name_set] = system('yarp namespace');
            if strcmp(name_set(17:end-1),robot.nameSpace) == 0
                [~,namespace] = system(['yarp namespace ' robot.nameSpace]);
                text = [text namespace];
                [~,detect] = system('yarp detect --write');
                text = [text detect];
            else
                text = [text name_set];
            end
            % Add configuration WBI
            text = [text robot.setupWBI(codyco_folder, build_folder)];
            % Add JOINT FRICTION in yarpWholeBodyInterface.ini
            if isWBIFrictionJoint(robot)
                robot.loadYarpWBI(codyco_folder, build_folder);
            end
            % Set variables environment
            assignin('base', 'Ts', robot.Ts);
            assignin('base', 'robotName', robot.robotName);
            assignin('base', 'localName', robot.localName);
            assignin('base', 'ROBOT_DOF', robot.ROBOT_DOF);
            assignin('base', 'YARP_ROBOT_NAME', robot.robotName);
            assignin('base', 'YARP_DATA_DIRS', [codyco_folder '/' build_folder '/install/share/codyco']);
            disp(text);
            else
                disp('You does not load anything motors!');
            end
        end
    end
    
    methods
        function text = setupWBI(robot, codyco_folder, build_folder)
            name_yarp_file = [build_folder '/main/WBIToolbox/share/codyco/contexts/wholeBodyInterfaceToolbox/wholeBodyInterfaceToolbox.ini'];
            text = sprintf('robot          %s\n',robot.robotName);
            text = [text sprintf('localName      %s\n',robot.localName)];
            text = [text sprintf('worldRefFrame  %s\n',robot.worldRefFrame)];
            text = [text sprintf('robot_fixed    %s\n',robot.robot_fixed)];
            text = [text sprintf('wbi_id_list    %s\n',robot.WBI_LIST)];
            text = [text sprintf('wbi_config_file %s', robot.configFile)];
            fid = fopen([codyco_folder '/' name_yarp_file],'w');
            fprintf(fid, '%s', text);
            fclose(fid);
        end
        
        function loadYarpWBI(robot, codyco_folder, build_folder)
            %% Load and save in BUILD directory configuraton
            copy_yarp_file = ['libraries/yarpWholeBodyInterface/app/robots/' robot.robotName '/yarpWholeBodyInterface.ini'];
            name_yarp_file = [build_folder '/install/share/codyco/robots/' robot.robotName '/yarpWholeBodyInterface.ini'];
            copyfile([codyco_folder '/' copy_yarp_file],[codyco_folder '/' name_yarp_file]);
            fid = fopen([codyco_folder '/' name_yarp_file], 'a+');
            command = [robot.WBI_LIST ' = (' robot.joint_list ')'];
            fprintf(fid, '# TEST JOINT\n%s', command);
            fclose(fid);
        end
    end
    
end

