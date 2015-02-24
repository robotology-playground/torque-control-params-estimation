classdef Motor
    %MOTOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        q;
        qdot;
        torque;
        pwm;
        voltage;
        current;
        time;
        friction_model;
        % Information about motor
        name_motor;
        Voltage;
        range_pwm;
        n_matrix;
    end
    
    properties
        Kt;
        friction;
        ratio = 1;
    end
    
    methods
        function motor = Motor(name_motor, n_matrix)
            motor.name_motor = name_motor;
            motor.n_matrix = n_matrix;
        end
        
        function motor = loadParameter(motor, file)
            text = Robot.parseFile(file, 'MOTOR_LIST_PARAMETERS');
            for i=1:size(text,2)
                [~,tok] = regexp(text{i}, '(\w+).*? = \((\w+).*?,(\w+).*?\)', 'match','tokens');
                tok = tok{:};
                if strcmp(tok{1},motor.name_motor)
                    motor = motor.setRatio(tok{2},tok{3});
                end
            end
        end
        
        function motor = setRatio(Voltage, range_pwm)
            motor.Voltage = str2double(Voltage); 
            motor.range_pwm = str2double(range_pwm);
            motor.ratio = motor.Voltage/motor.range_pwm;
        end
        
        function motor = loadMeasure(motor, data, part, number)
            %% Load measure to add in motor
            motor.q = data.q;
            motor.qdot = data.qD;
            motor.torque = data.tau;
            temp_pwm = data.PWM.(part);
            motor.pwm = temp_pwm(:,number);
            motor.voltage = motor.ratio*motor.pwm;
            if isfield(data,'current')
                motor.current = data.current;
            end
            motor.time = data.time;
            if size(motor.friction,1) > 0
                motor = motor.evaluateCoefficient();
            end
        end
        
        function plotCollected(motor)
            %% Plot Collected images - Motor and model
            hold on
            motor.plotMotor();
            motor.plotMotorModel();
            hold off
        end
        
        function plotMotorModel(motor)
            %% Plot Motor model
            title(sprintf('Kt: %12.8f [Nm]/[V] - Name motor: %s',motor.Kt, motor.name_motor));
            plot(motor.voltage , motor.voltage*motor.Kt,'r-','LineWidth',3);
        end
        
        function plotMotor(motor, option)
            %% Plot measure versus friction estimation
            if ~exist('option','var')
                option = '.';
            end
            plot(motor.voltage, motor.torque-motor.friction_model, option);
            if motor.ratio == 1
                xlabel('PWM','Interpreter','tex');
            else
                xlabel('Voltage','Interpreter','tex');
            end
            ylabel('\tau-\tau_{f}','Interpreter','tex');
        end
        
        function motor = evaluateCoefficient(motor)
            %% Evaluate coefficient for Kt motor
            motor.friction_model = motor.friction.getFriction(motor.qdot);
            A = Joint.linearRegression(motor.voltage,motor.torque-motor.friction_model);
            %A = joint.linearRegression(joint.current,joint.torque-joint.friction_model);
            motor.Kt = A(1);
        end

        function text = saveCoeffToFile(motor)
            %% Write information to txt file
            text = '';
            if size(motor.Kt,1) ~= 0
                text = [text, sprintf('\n----------> Kt <----------\n')];
                text = [text, sprintf('tau_m = Kt*PWM \n')];
                text = [text, sprintf('Kt: %12.8f [Nm]/[V]',motor.Kt)];
            end
        end
        
        function text = saveLatexCoeffToFile(motor)
            %% Get Latex File
            text = sprintf('\n---- Kt -> Latex ----\n');
            text = [text, sprintf('\n\\begin{equation}\n')];
            text = [text, sprintf('\\label{eq:%sCoeffPWM}\n',motor.name_motor)];
            text = [text, sprintf('\\begin{array}{cccl}\n')];
            text = [text, sprintf('\\bar Kt & \\simeq & %12.8f & \\frac{[Nm]}{[V]}\n',motor.Kt)];
            text = [text, sprintf('\\end{array}\n')];
            text = [text, sprintf('\\end{equation}\n')];
        end

        function text = textControlData(motor)
            %% Information joint estimation
            text = '';
            if size(motor.Kt,1) ~= 0
                text = sprintf('Control Friction\n');
                text = [text sprintf('PWM = kt tau - [s(q)(kc+ + kv+ qdot q) + s(-q)(kc- + kv- qdot q)]\n')];
                text = [text sprintf('kc+: %12.8f [V] - kc-: %12.8f [V] \n',motor.friction.KcP/motor.Kt, motor.friction.KcN/motor.Kt)];
                text = [text sprintf('kv+: %12.8f [V][s]/[deg] - kv-: %12.8f [V][s]/[deg]\n',motor.friction.KvP/motor.Kt, motor.friction.KvN/motor.Kt)];
                %fprintf(fileID,'KsP: %12.8f [Nm] - KsN %12.8f [Nm][s]/[deg]\n',joint.friction.KsP, joint.friction.KsN);
                text = [text sprintf('kt: %12.8f [V]/[Nm]\n',1/motor.Kt)];
            end
        end
    end
    
end

