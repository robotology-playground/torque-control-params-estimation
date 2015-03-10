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
        maxTorque;
        % Kt values
        Kt;
        n_matrix;
    end
    
    properties
        name;
        friction;
        ratioTorque = 1;
        ratioVoltage = 1;
        ktau = 0
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
                [~,tok] = regexp(text{i}, '(\w+).*? = \((\w+).*?,(\w+).*?,(\w+).*?,(\w+).*?\)', 'match','tokens');
                tok = tok{:};
                if strcmp(tok{1},motor.name)
                    motor = motor.setRatioVoltage(str2double(tok{2}),str2double(tok{3}));
                    motor = motor.setRatioTorque(str2double(tok{4}),str2double(tok{5}));
                end
            end
        end
        
        function Kt = getKt(motor)
            Kt = motor.Kt;
        end
        
        function motor = setRatioTorque(motor, maxTorque, maxInt)
            motor.maxTorque = maxTorque;
            motor.ratioTorque = motor.maxTorque/maxInt;
        end
        
        function motor = setRatioVoltage(motor, Voltage, range_pwm)
            motor.Voltage = Voltage; 
            motor.range_pwm = range_pwm;
            motor.ratioVoltage = motor.Voltage/motor.range_pwm;
        end
        
        function motor = loadMeasure(motor, data, part, number)
            %% Load measure to add in motor
            motor.q = data.q;
            motor.qdot = data.qD;
            motor.torque = data.tau./motor.ratioTorque;
            temp_pwm = data.PWM.(part);
            motor.pwm = temp_pwm(:,number);
            motor.voltage = motor.ratioVoltage*motor.pwm;
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
            if motor.ratioVoltage ~= 1
                unitX = 'V';
            else
                unitX = 'PWM';
            end
            if motor.ratioTorque ~= 1
                unitY = 'Nm';
            else
                unitY = 'TorqueMachine';
            end
            title(sprintf('Kt: %12.8f [%s]/[%s] - Name motor: %s',motor.Kt, unitY, unitX, motor.name));
            plot(motor.voltage , motor.voltage*motor.Kt,'r-','LineWidth',3);
        end
        
        function plotMotor(motor, option)
            %% Plot measure versus friction estimation
            if ~exist('option','var')
                option = '.';
            end
            plot(motor.voltage, motor.torque-motor.friction_model, option);
            if motor.ratioVoltage == 1
                xlabel('PWM','Interpreter','tex');
            else
                xlabel('Voltage','Interpreter','tex');
            end
            if motor.ratioVoltage == 1
                ylabel('TorqueMachine','Interpreter','tex');
            else
                ylabel('\tau-\tau_{f}','Interpreter','tex');
            end
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
                if motor.ratioVoltage ~= 1
                    unitX = 'V';
                else
                    unitX = 'PWM';
                end
                if motor.ratioTorque ~= 1
                    unitY = 'Nm';
                else
                    unitY = 'TorqueMachine';
                end
                text = [text, sprintf('Kt: %12.8f [%s]/[%s]',motor.Kt, unitY, unitX)];
                if motor.ratioVoltage ~= 1
                    text = [text, sprintf('\nRatio Volt: %12.8f [V]/PWM',motor.ratioVoltage)];
                end
                if motor.ratioTorque ~= 1
                    text = [text, sprintf('\nRatio Torque: %12.8f [Nm]/TorqueMachine',motor.ratioTorque)];
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
            motor.ktau = 1/motor.Kt;
            motor.stictionUp = sign(motor.ktau)*abs(motor.friction.KcP/motor.Kt);
            motor.stictionDown = sign(motor.ktau)*abs(mean([motor.friction.KvP/motor.Kt,motor.friction.KvN/motor.Kt]));
            motor.bemf = sign(motor.ktau)*abs(motor.friction.KvP/motor.Kt);
        end

        function text = textControlData(motor)
            %% Information joint estimation
            
            motor = motor.controlValue();
            text = '';
            if size(motor.Kt,1) ~= 0
                if motor.ratioVoltage ~= 1
                    unitX = 'V';
                else
                    unitX = 'PWM';
                end
                if motor.ratioTorque ~= 1
                    unitY = 'Nm';
                else
                    unitY = 'TorqueMachine';
                end
                text = sprintf('\n---------->  Parameters for joint torque control  <----------\n\n');
                text = [text sprintf('ktau:\t\t\t%12.8f [%s]/[%s]\n',motor.ktau,unitX,unitY)];
                text = [text sprintf('stictionUp:\t\t%12.8f [%s]\n', motor.stictionUp,unitX)];
                text = [text sprintf('stictionDown:\t%12.8f [%s]\n', motor.stictionDown,unitX)];
                text = [text sprintf('bemf:\t\t\t%12.8f [%s][s]/[deg]\n', motor.bemf,unitX)];
                %fprintf(fileID,'KsP: %12.8f [Nm] - KsN %12.8f [Nm][s]/[deg]\n',joint.friction.KsP, joint.friction.KsN);
                
            end
        end
    end
    
end

