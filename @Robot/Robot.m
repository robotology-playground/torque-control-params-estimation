classdef Robot
    %ROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        robot_type = 'icub';
        path_experiment;
        joint_list = '';
        nameSpace;
        formatOut = 'yyyymmdd-HH:MM';
        worldRefFrame = 'root_link';
        robot_fixed = 'true';
        configFile = 'yarpWholeBodyInterface.ini';
    end
    
    properties
        localName = 'simulink_joint_friction';
        nameRobot;
        path;
        Ts = 0.01;
        joints;
    end
    
    methods
        function robot = Robot(nameRobot, path_experiment)
            robot.nameRobot = nameRobot;
            robot.nameSpace = [ '/' robot.robot_type nameRobot(end-1:end)];
            if ~exist('path_experiment','var')
                robot.path_experiment = 'experiments';
            else
                robot.path_experiment = path_experiment;
            end
            robot.path = [robot.path_experiment '/' robot.nameRobot '/'];
        end
        
        function robot = setNameSpace(nameSpace)
            %% Configure type of namespace
            robot.nameSpace = nameSpace;
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
        
        function robot = addMotor(robot, part, type, info1, info2)
            %% Add motor in robot
            if exist('type','var') && exist('info1','var') && exist('info2','var')
                motor = Motor(robot.path_experiment, robot.nameRobot, part, type, info1, info2);
            elseif exist('type','var') && exist('info1','var') && ~exist('info2','var')
                motor = Motor(robot.path_experiment, robot.nameRobot, part, type, info1);
            elseif exist('type','var') && ~exist('info1','var') && ~exist('info2','var')
                motor = Motor(robot.path_experiment, robot.nameRobot, part, type);
            elseif ~exist('type','var') && ~exist('info1','var') && ~exist('info2','var')
                motor = Motor(robot.path_experiment, robot.nameRobot, part);
            end
            robot.joints = [robot.joints motor];
            
            if strcmp(robot.joint_list,'')
                robot.joint_list = motor.WBIname;
            else
                robot.joint_list = [robot.joint_list ', ' motor.WBIname];
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
            [~,namespace] = system(['yarp namespace ' robot.nameSpace]);
            text = [text namespace];
            [~,detect] = system('yarp detect --write');
            text = [text detect];
            % Add configuration WBI
            robot.setupWBI(codyco_folder, build_folder);
            % Add JOINT FRICTION in yarpWholeBodyInterface.ini
            robot.loadYarpWBI(codyco_folder, build_folder);
            % Set variables environment
            assignin('base', 'ROBOT_DOF', size(robot.joints,2));
            assignin('base', 'YARP_ROBOT_NAME', robot.nameRobot);
            %assignin('base', 'YARP_DATA_DIRS', [codyco_folder '/' build_folder '/install/share/codyco']);
            disp(text);
            else
                disp('You does not load motor!');
            end
        end
    end
    
    methods
        function setupWBI(robot, codyco_folder, build_folder)
            name_yarp_file = [build_folder '/main/WBIToolbox/share/codyco/contexts/wholeBodyInterfaceToolbox/wholeBodyInterfaceToolbox.ini'];
            text = sprintf('robot          %s\n',robot.nameRobot);
            text = [text sprintf('localName      %s\n',robot.localName)];
            text = [text sprintf('worldRefFrame  %s\n',robot.worldRefFrame)];
            text = [text sprintf('robot_fixed    %s\n',robot.robot_fixed)];
            text = [text sprintf('wbi_id_list    JOINT_FRICION\n')];
            text = [text sprintf('wbi_config_file %s', robot.configFile)];
            fid = fopen([codyco_folder '/' name_yarp_file]);
            fprintf(fid, '%s', text);
            fclose(fid);
        end
        
        function loadYarpWBI(robot, codyco_folder, build_folder)
            %% Load and save in BUILD directory configuraton
            copy_yarp_file = ['libraries/yarpWholeBodyInterface/app/robots/' robot.nameRobot '/yarpWholeBodyInterface.ini'];
            name_yarp_file = [build_folder '/install/share/codyco/robots/' robot.nameRobot '/yarpWholeBodyInterface.ini'];
            copyfile([codyco_folder '/' copy_yarp_file],[codyco_folder '/' name_yarp_file]);
            fid = fopen([codyco_folder '/' name_yarp_file], 'a+');
            command = ['JOINT_FRICTION = (' robot.joint_list ')'];
            fprintf(fid, '# TEST JOINT\n%s', command);
            fclose(fid);
        end
    end
    
end

