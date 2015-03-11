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
        Voltage;
        range_pwm;
        % Kt values
        Kt;
        n_matrix;
    end
    
    properties
        name;
        friction;
        ratio = 1;
        kff = 0
        stictionUp = 0;
        stictionDown = 0;
        bemf = 0;
    end
    
    methods
        function motor = Motor(name, n_matrix)
            motor.name = name;
            motor.n_matrix = n_matrix;
        end
        
        function motor = loadParameter(motor, file)
            text = Robot.parseFile(file, 'MOTOR_LIST_PARAMETERS');
            for i=1:size(text,2)
                [~,tok] = regexp(text{i}, '(\w+).*? = \((\w+).*?,(\w+).*?\)', 'match','tokens');
                tok = tok{:};
                if strcmp(tok{1},motor.name)
                    motor = motor.setRatio(tok{2},tok{3});
                end
            end
        end
        
        function Kt = getKt(motor)
            Kt = motor.Kt;
        end
        
        function motor = setRatio(motor, Voltage, range_pwm)
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
            if motor.ratio ~= 1
                unit = 'V';
            else
                unit = 'PWM';
            end
            title(sprintf('Kt: %12.8f [Nm]/[%s] - Name motor: %s',motor.Kt, unit, motor.name));
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
                text = [text, sprintf('\n---------->     Kt    <----------\n')];
                text = [text, sprintf('tau_m = Kt*PWM \n')];
                if motor.ratio ~= 1
                    unit = 'V';
                else
                    unit = 'PWM';
                end
                text = [text, sprintf('Kt: %12.8f [Nm]/[%s]',motor.Kt, unit)];
                if motor.ratio ~= 1
                    text = [text, sprintf('\nRatio: %12.8f [V]/PWM',motor.ratio)];
                end
            end
        end
        
        function text = saveLatexCoeffToFile(motor)
            %% Get Latex File
            text = sprintf('\n---- Kt -> Latex ----\n');
            text = [text, sprintf('\n\\begin{equation}\n')];
            text = [text, sprintf('\\label{eq:%sCoeffPWM}\n',motor.name)];
            text = [text, sprintf('\\begin{array}{cccl}\n')];
            text = [text, sprintf('\\bar Kt & \\simeq & %12.8f & \\frac{[Nm]}{[V]}\n',motor.Kt)];
            text = [text, sprintf('\\end{array}\n')];
            text = [text, sprintf('\\end{equation}\n')];
        end
        
        function motor = controlValue(motor)
            %% Control values
            motor.kff = 1/motor.Kt;
            motor.stictionUp = sign(motor.kff)*abs(motor.friction.KcP/motor.Kt);
            motor.stictionDown = sign(motor.kff)*abs(motor.friction.KcN/motor.Kt);
            motor.bemf = sign(motor.kff)*abs(mean([motor.friction.KvP/motor.Kt,motor.friction.KvN/motor.Kt]));
        end

        function text = textControlData(motor)
            %% Information joint estimation
            
            motor = motor.controlValue();
            text = '';
            if size(motor.Kt,1) ~= 0
                if motor.ratio ~= 1
                    unit = 'V';
                else
                    unit = 'PWM';
                end
                text = sprintf('\n---------->  Parameters for joint torque control  <----------\n\n');
                text = [text sprintf('kff:\t\t\t%12.8f [%s]/[Nm]\n',motor.kff,unit)];
                text = [text sprintf('stictionUp:\t\t%12.8f [%s]\n', motor.stictionUp,unit)];
                text = [text sprintf('stictionDown:\t%12.8f [%s]\n', motor.stictionDown,unit)];
                text = [text sprintf('bemf:\t\t\t%12.8f [%s][s]/[deg]\n', motor.bemf,unit)];
                %fprintf(fileID,'KsP: %12.8f [Nm] - KsN %12.8f [Nm][s]/[deg]\n',joint.friction.KsP, joint.friction.KsN);
                
            end
        end
    end
    
end

