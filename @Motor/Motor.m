classdef Motor
    %MOTOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        N_HEAD = 0;
        N_TORSO = 3;
        N_HAND = 5;
        N_LEG = 6;
        ANKLE_START = 4;
    end
    
    properties (Access = protected)
        start_folder;
        part;
        type;
        info1;
        info2;
        robot_dof = 0;
    end
    
    properties
        robot;
        number;
        number_part;
        path;
        group_select;
        friction;
        select;
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
            if nargin ~= 0
                if strcmp(varargin{1},'number_joint')
                    joint.robot_dof = varargin{2};
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
            end
            joint = joint.JointStructure(); % Build path
        end
        
        function joint = loadFrictionData(joint, position, velocity, torque, time, threshold, offset)
            if ~exist('threshold','var')
                threshold = 1;
            end
            if ~exist('offset','var')
                offset = 0;
            end
            joint.friction = Friction(position, velocity, torque, time, threshold, offset);
        end
        
        function joint = loadFriction(joint, file, threshold, offset)
            if ~exist('threshold','var')
                threshold = 1;
            end
            if ~exist('offset','var')
                offset = 0;
            end
            data = load([joint.folder_path file]);
            joint.friction = Friction(data.out(:,1) ,data.out(:,2),data.out(:,3), data.time, threshold,offset);
        end
        
        function measure = plotFrVsMeasure(joint, file, time_init, time_stop)
            
            measure = struct;
            data = load([joint.folder_path file]);
            
            measure.time = data.logsout.get('q').Values.Time;
            
            if ~exist('time_init','var')
                time_init = 0;
            end
            if ~exist('time_stop','var')
                time_stop = measure.time(end);
            else
                if strcmp(time_stop,'end')
                    time_stop = measure.time(end);
                end
            end
            measure.q = data.logsout.get('q').Values.Data(time_init*100+1:time_stop*100+1,joint.number);
            measure.qdot = data.logsout.get('qD').Values.Data(time_init*100+1:time_stop*100+1,joint.number);
            measure.torque = data.logsout.get('tau').Values.Data(time_init*100+1:time_stop*100+1,joint.number);
            measure.pwm = data.logsout.get('pwm').Values.Data(time_init*100+1:time_stop*100+1,joint.number_part);
            measure.current = data.logsout.get('current').Values.Data(time_init*100+1:time_stop*100+1,joint.number_part);
            measure.time = measure.time(time_init*100+1:time_stop*100+1);
            
            measure.friction_model = joint.friction.getFriction(measure.qdot);
            
            subplot(1,2,1);
            hold on
            joint.friction.plotFriction();
            grid;
            joint.friction.plotFrictionModel();
            hold off
            
            subplot(1,2,2);
            plot(measure.pwm,measure.torque-measure.friction_model,'.');
            xlabel('PWM','Interpreter','tex');
            ylabel('\tau','Interpreter','tex');
            grid;
            hold on
            measure.a = lineRegress(measure.current,measure.torque-measure.friction_model);
            %measure.a = lineRegress(measure.pwm,measure.torque-measure.friction_model);
            %plot(measure.pwm,measure.pwm*measure.a(1),'r-','LineWidth',3);
            plot(measure.current,measure.current*measure.a(1),'r-','LineWidth',3);
        end
    end
    
    methods (Access = protected)
        function joint = buildFolder(joint)
            % Build a folder path and if doesn't exist a folder build a
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

