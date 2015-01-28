classdef Motor
    %MOTOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = protected)
        robot;
        part;
        type;
        info1;
        info2;
    end
    
    properties
        number;
        path;
        group_select;
        friction;
        robotName;
        select;
        figureName;
    end
    
    methods
        function joint = Motor(start_folder, robot, part, type, info1, info2)
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
            % TODO remove
            % Build folder
            joint = joint.buildFolder(start_folder);
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
        
        function measure = plotFrVsMeasure(joint,file, time_init, time_stop)
            
            measure = struct;
            data = load([joint.folder_path file]);
            
            if strcmp(time_stop,'end')
                time_stop = data.time(end);
            end
            measure.q = data.logsout.get('q').Values.Data(:,4);
            measure.qdot = data.logsout.get('qD').Values.Data(:,4);
            measure.torque = data.logsout.get('tau').Values.Data(:,4);
            measure.pwm = data.logsout.get('pwm').Values.Data;
            measure.current = data.logsout.get('current').Values.Data;
            measure.time = data.time;
            
            %             measure.time = data.time(time_init*100+1:time_stop*100+1);
            %             measure.qdot = data.out(time_init*100+1:time_stop*100+1,2);
            %             measure.torque = data.out(time_init*100+1:time_stop*100+1,3);
            %             measure.pwm = data.out(time_init*100+1:time_stop*100+1,4);
            %             measure.pwm_jtc = data.out(time_init*100+1:time_stop*100+1,5);
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
        function joint = buildFolder(joint, start_folder)
            % Build a folder path and if doesn't exist a folder build a
            % selected forlder from information type of joint
            joint = joint.JointStructure(start_folder); % Build path
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

