classdef Robot
    %ROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        robot_type = 'icub';
        path_experiment;
        joint_list = '';
        nameSpace;
        joints;
        time_acquisition = 0.01;
    end
    
    properties
        nameRobot;
        path;
    end
    
    methods
        function robot = Robot(nameRobot, path_experiment)
            robot.nameRobot = nameRobot;
            robot.nameSpace = [ '/' robot.robot_type nameRobot(end-1:end)];
            robot.path_experiment = path_experiment;
            robot.path = [robot.path_experiment '/' robot.nameRobot '/'];
        end
        
        function robot = setNameSpace(nameSpace)
            robot.nameSpace = nameSpace;
        end
        
        function robot = addMotor(robot, part, type, info1, info2)
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
        
        function text = configure(robot, codyco_folder, build_folder)
            if ~exist('build_folder','var')
                build_folder = 'build';
            end
            text = '';
            [~,namespace] = system(['yarp namespace ' robot.nameSpace]);
            text = [text namespace];
            [~,detect] = system('yarp detect --write');
            text = [text detect];
            % Add JOINT FRICTION in yarpWholeBodyInterface.ini
            robot.loadYarpWBI(codyco_folder, build_folder);
            % Set variables environment
            assignin('base', 'ROBOT_DOF', size(robot.joint,2));
            assignin('base', 'YARP_ROBOT_NAME', robot.nameRobot);
            assignin('base', 'robotName', robot.robot_type);
            assignin('base', 'localName', 'simulink_joint_friction');
            assignin('base', 'YARP_DATA_DIRS', [codyco_folder '/' build_folder '/install/share/codyco']);
            assignin('base', 'Ts', robot.time_acquisition);
        end
    end
    
    methods
        function loadYarpWBI(robot, codyco_folder, build_folder)
            copy_yarp_file = ['libraries/yarpWholeBodyInterface/app/robots/' robot.nameRobot '/yarpWholeBodyInterface.ini'];
            name_yarp_file = [build_folder '/install/share/codyco/robots/' robot.nameRobot '/yarpWholeBodyInterface.ini'];
            copyfile([codyco_folder copy_yarp_file],[codyco_folder name_yarp_file]);
            fid = fopen([codyco_folder name_yarp_file], 'a+');
            command = ['JOINT_FRICTION = (' robot.joint_list ')'];
            fprintf(fid, '# TEST JOINT\n%s', command);
            fclose(fid);
        end
    end
    
end

