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
        measure;
        friction_model;
        
        a;
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
        
        function joint = loadIdleMeasureData(joint, position, velocity, torque, time, threshold, offset)
            if ~exist('threshold','var')
                threshold = 1;
            end
            joint.friction = Friction(position, velocity, torque, time, threshold);
        end
        
        function joint = loadIdleMeasure(joint, file, threshold)
            if ~exist('file','var')
                file = 'idle.mat';
            end
            if ~exist('threshold','var')
                threshold = 1;
            end
            data = load([joint.path file]);
            if size(data.logsout.get('q').Values.Data,2) == 25
                numb = joint.number;
            else
                numb = 1;
            end
            position_data = data.logsout.get('q').Values.Data(:,numb);
            velocity_data = data.logsout.get('qD').Values.Data(:,numb);
            torque_data = data.logsout.get('tau').Values.Data(:,numb);
            joint.friction = Friction(position_data, velocity_data, torque_data, data.time, threshold);
        end
        
        function joint = loadReference(joint, file)
            if ~exist('file','var')
                file = 'reference.mat';
            end
            data = load([joint.path file]);
            joint.q = data.logsout.get('q').Values.Data(:,joint.number);
            joint.qdot = data.logsout.get('qD').Values.Data(:,joint.number);
            joint.torque = data.logsout.get('tau').Values.Data(:,joint.number);
            joint.pwm = data.logsout.get('pwm').Values.Data(:,joint.number_part);
            joint.current = data.logsout.get('current').Values.Data(:,joint.number_part);
            joint.time = data.logsout.get('q').Values.Time;
            joint.friction_model = joint.friction.getFriction(joint.measure.qdot);
        end
        
        function joint = plotFrVsMeasure(joint)
            
            subplot(1,2,1);
            hold on
            joint.friction.plotFriction();
            grid;
            joint.friction.plotFrictionModel();
            hold off
            
            subplot(1,2,2);
            plot(joint.pwm, joint.torque-joint.friction_model,'.');
            xlabel('PWM','Interpreter','tex');
            ylabel('\tau','Interpreter','tex');
            grid;
            hold on
            joint.a = lineRegress(joint.current,joint.torque-joint.friction_model);
            %measure.a = lineRegress(measure.pwm,measure.torque-measure.friction_model);
            %plot(measure.pwm,measure.pwm*measure.a(1),'r-','LineWidth',3);
            plot(joint.current,joint.current*joint.a(1),'r-','LineWidth',3);
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

