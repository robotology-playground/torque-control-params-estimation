classdef Motor
    %MOTOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        number;
        path;
        folder_path;
        friction;
        robotName;
    end
    
    methods
        function joint = Motor(robot, part, type, info1, info2)
            joint = joint.JointStructure(robot, part, type, info1, info2);
            if ~exist(joint.folder_path,'dir')
                mkdir(joint.folder_path);
            end
        end
        
        function joint = loadFriction(joint, file, threshold)
            data = load([joint.folder_path file]);
            joint.friction = Friction(data.out(:,1) ,data.out(:,2),data.out(:,3), data.time, threshold);
        end
        
        function a = plotFrVsMeasure(joint,file, time_init, time_stop)
            
            data = load([joint.folder_path file]);
            
            if strcmp(time_stop,'end')
                time_stop = data.time(end);
            end
            
            qdot = data.out(time_init*100+1:time_stop*100+1,2);
            torque = data.out(time_init*100+1:time_stop*100+1,3);
            pwm = data.out(time_init*100+1:time_stop*100+1,4);
            %pwm_jtc = data.out(time_init*100+1:time_stop*100+1,5);
            friction_model = joint.friction.getFriction(qdot);
            
            
            subplot(1,2,1);
            hold on
            joint.friction.plotFriction();
            grid;
            joint.friction.plotFrictionModel();
            hold off
            
            subplot(1,2,2);
            plot(pwm,torque-friction_model,'.');
            xlabel('PWM','Interpreter','tex');
            ylabel('\tau','Interpreter','tex');
            grid;
            hold on
            a = lineRegress(pwm,torque-friction_model);
            plot(pwm,pwm*a(1),'r-');
            
        end
    end
    
end

