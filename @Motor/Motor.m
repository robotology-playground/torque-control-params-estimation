classdef Motor
    %MOTOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        number;
        path;
        folder_path;
        friction;
        robotName;
        select;
        figureName;
        name_exp;
    end
    
    methods
        function joint = Motor(start_folder, robot, part, type, info1, info2,name_exper)
            if ~exist('info1','var')
                info1 = '';
            end
            if ~exist('info2','var')
                info2 = '';
            end
            if ~exist('name_exper','var')
                name_exper = '';
            end
            joint = joint.JointStructure(start_folder, robot, part, type, info1, info2,name_exper);
            if ~exist(joint.folder_path,'dir')
                mkdir(joint.folder_path);
            end
            joint.name_exp = name_exper;
        end
        
        function joint = loadFriction(joint, file, threshold,offset)
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
            
            measure.time = data.time(time_init*100+1:time_stop*100+1);
            measure.qdot = data.out(time_init*100+1:time_stop*100+1,2);
            measure.torque = data.out(time_init*100+1:time_stop*100+1,3);
            measure.pwm = data.out(time_init*100+1:time_stop*100+1,4);
            measure.pwm_jtc = data.out(time_init*100+1:time_stop*100+1,5);
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
            measure.a = lineRegress(measure.pwm,measure.torque-measure.friction_model);
            plot(measure.pwm,measure.pwm*measure.a(1),'r-','LineWidth',3);
            
        end
    end
    
end

