classdef Motor
    %MOTOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        N_HEAD = 0;
        N_TORSO = 3;
        N_HAND = 5;
        N_LEG = 6;
        ANKLE_START = 4;
        
        q;
        qdot;
        torque;
        pwm;
        current;
        time;
        path_before;
    end
    
    properties (Access = protected)
        start_folder;
        part;
        type;
        info1;
        info2;
        robot_dof = 0;
        measure;
        friction_model;
    end
    
    properties
        robot;
        number;
        number_part;
        path;
        group_select;
        friction;
        select;
        Kt;
        WBIname;
    end
    
    methods
        function joint = Motor(start_folder, robot, part, type, info1, info2)
            joint.start_folder = start_folder;
            joint.robot = robot;
            joint.part = part;
            if exist('type','var')
                joint.type = type;
            end
            if exist('info1','var')
                joint.info1 = info1;
            end
            if exist('info2','var')
                joint.info2 = info2;
            end
            joint.robot_dof = 25;
            % TODO remove
            % Build folder
            joint = joint.buildFolder();
        end
        
        
        function joint = setPart(joint, varargin)
            %% Set part avaiable on robot
            if nargin ~= 0
                joint = joint.JointStructure(); % Build path
                if strcmp(varargin{1},'number_joint')
                    joint.robot_dof = varargin{2};
                    joint.number = varargin{2};
                end
            else
                for i=1:nargin
                    type_arg = varargin{i};
                    if strcmp(type_arg,'head')
                        joint.robot_dof = joint.robot_dof + joint.N_HEAD;
                    elseif strcmp(type_arg,'torso')
                        joint.robot_dof = joint.robot_dof + joint.N_TORSO;
                    elseif strcmp(type_arg,'left_arm')
                        joint.robot_dof = joint.robot_dof + joint.N_HAND;
                    elseif strcmp(type_arg,'left_leg')
                        joint.robot_dof = joint.robot_dof + joint.N_LEG;
                    elseif strcmp(type_arg,'right_arm')
                        joint.robot_dof = joint.robot_dof + joint.N_HAND;
                    elseif strcmp(type_arg,'right_leg')
                        joint.robot_dof = joint.robot_dof + joint.H_LEG;
                    end
                end
                joint = joint.JointStructure(); % Build path
            end
        end
        
        function joint = loadIdleMeasureData(joint, position, velocity, torque, time, threshold, cutoff)
            if ~exist('threshold','var')
                threshold = 1;
            end
            if exist('cutoff','var')
                joint.friction = Friction(position, velocity, torque, time, threshold, cutoff);
            else
                joint.friction = Friction(position, velocity, torque, time, threshold);
            end
        end
        
        function joint = loadIdleMeasure(joint, file, threshold, cutoff)
            %% Load data from mat file
            if ~exist('file','var')
                file = 'idle';
            end
            if ~exist('threshold','var')
                threshold = 1;
            end
            data = load([joint.path file '.mat']);
            if exist('cutoff','var')
                joint.friction = Friction(data.q, data.qD, data.tau, data.time, threshold, cutoff);
                joint.friction = joint.friction.setExperiment(file);
            else
                joint.friction = Friction(data.q, data.qD, data.tau, data.time, threshold);
                joint.friction = joint.friction.setExperiment(file);
            end
        end
        
        function joint = loadReference(joint, data)
            joint.q = data.q;
            joint.qdot = data.qD;
            joint.torque = data.tau;
            temp_pwm = data.PWM.(joint.group_select);
            joint.pwm = temp_pwm(:,joint.number_part);
            if isfield(data,'current')
                joint.current = data.current;
            end
            joint.time = data.time;
            joint.friction_model = joint.friction.getFriction(joint.qdot);
            joint = joint.evaluateCoeff();
        end
        
        function joint = loadRefFile(joint, file)
            %% Load reference from file
            if ~exist('file','var')
                file = 'reference';
            end
            data = load([joint.path file '.mat']);
            joint = joint.loadReference(data);
        end
        
        function joint = evaluateCoeff(joint)
            A = joint.linearRegression(joint.pwm,joint.torque-joint.friction_model);
            %A = joint.linearRegression(joint.current,joint.torque-joint.friction_model);
            joint.Kt = A(1);
        end
        
        
        function joint = plotCoeff(joint, option)
            %% Plot measure versus friction estimation
            if ~exist('option','var')
                option = '.';
            end
            plot(joint.pwm, joint.torque-joint.friction_model, option);
            xlabel('PWM','Interpreter','tex');
            ylabel('\tau-\tau_{f}','Interpreter','tex');
            hold on;
            plot(joint.pwm , joint.pwm*joint.Kt,'r-','LineWidth',3);
            hold off;
            %plot(joint.current,joint.current*joint.a(1),'r-','LineWidth',3);
        end
        
        
        function savePictureToFile(joint,hFig,figureName)
            %% Save Friction picture
            % Save image
            currentFolder = pwd;
            cd(joint.path);
            if ~exist('figureName','var')
                figureName = 'friction';
            end
            saveas(hFig,[figureName '.fig'],'fig');
            saveas(hFig,[figureName '.png'],'png');
            cd(currentFolder);
            clear currentFolder;
        end
        
        function command = getControlBoardCommand(joint, rate)
            %% Get string to start ControlBoardDumper
            if ~exist('rate','var')
                rate = 10;
            end
            command = ['controlBoardDumper'...
                ' --robot icub'...
                ' --part ' joint.group_select ...
                ' --rate ' num2str(rate) ...
                ' --joints "(' num2str(joint.number_part-1) ')"' ...
                ' --dataToDump "(getOutputs getCurrents)"'];
        end
        
        function command = getWBIlist(joint)
            %% Get string to start ControlBoardDumper
            command = ['JOINT_FRICTION = (' joint.WBIname ')'];
            assignin('base', 'ROBOT_DOF', 1);
        end
        
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
        
        function text = saveCoeffToFile(joint)
            %% Write information to txt file
            
            % Information joint estimation
            text = sprintf('Name: %s\n',joint.robot);
            text = [text, sprintf('Part: %s\n',joint.part)];
            if(joint.type ~= 0)
                text = [text, sprintf('Type: %s\n',joint.type)];
            end
            if(joint.info1 ~= 0)
                text = [text, sprintf('Info1: %s\n',joint.info1)];
            end
            if(joint.info2 ~= 0)
                text = [text, sprintf('Info2: %s\n',joint.info2)];
            end
            text = [text, sprintf('%s\n',joint.friction.saveToFile(joint.path))];
            text = [text, sprintf('\n----------> Kt <----------\n')];
            text = [text, sprintf('Kt: %12.8f [Nm]/[V]\n',joint.Kt)];
            text = [text, sprintf('\n---- Kt -> Latex ----\n')];
            text = [text, sprintf('\n\\begin{equation}\n')];
            text = [text, sprintf('\\label{eq:%sCoeffPWM}\n',joint.path)];
            text = [text, sprintf('\\begin{array}{cccl}\n')];
            text = [text, sprintf('\\bar Kt & \\simeq & %12.8f & \\frac{[Nm]}{[V]}\n',joint.friction.KvP)];
            text = [text, sprintf('\\end{array}\n')];
            text = [text, sprintf('\\end{equation}\n')];
        end
        
        function joint = saveControlToFile(joint, name)
            %% Save information to txt file
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
        
        
        function path = getPathType(joint)
            %% Get path type
            path = joint.path_before;
        end
    end
    
    methods (Access = protected)
        function joint = buildFolder(joint)
            %% Build a folder path and if doesn't exist a folder build a
            % selected forlder from information type of joint
            joint = joint.JointStructure(); % Build path
            if ~exist(joint.path,'dir') % Build folder
                mkdir(joint.path);
            end
        end
    end
    
    methods (Access = protected, Static)
        function a = linearRegression(x, y)
            N = size(x,1);
            % Add column of 1's to include constant term in regression
            X = [x ones(N,1)];
            % = [a1; a0]
            a = regress(y,X);
        end
        function number = pitchRollYawNumber(info)
            number = 0;
            if strcmp(info,'pitch')
                number = 1;
            elseif strcmp(info,'roll')
                number = 2;
            elseif strcmp(info,'yaw')
                number = 3;
            end
        end
    end
    
end

