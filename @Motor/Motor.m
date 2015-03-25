classdef Motor
    %MOTOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        q;
        qdot;
        torque;
        pwm;
        voltage;
        torqueMachine;
        current;
        time;
        friction_model;
        % Information about motor
        Voltage;
        range_pwm;
        maxTorque;
        % Kt values
        Kt;
        KtFirmware;
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
                [~,tok] = regexp(text{i}, '(\w+).*? = \((\w+).*?,(\w+).*?,(\w+).*?\)', 'match','tokens');
                tok = tok{:};
                if strcmp(tok{1},motor.name)
                    motor = motor.setRatioVoltage(str2double(tok{2}),str2double(tok{3}));
                    motor = motor.setRatioTorque(str2double(tok{4}));
                end
            end
        end
        
        function Kt = getKt(motor)
            Kt = motor.Kt;
        end
        
        function Kt = getKtFirmware(motor)
            Kt = motor.KtFirmware;
        end
        
        function motor = setRatioTorque(motor, maxTorque)
            if maxTorque ~= 1
                motor.maxTorque = maxTorque;
                motor.ratioTorque = 32000/motor.maxTorque;
            end
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
            motor.torque = data.tau;
            temp_pwm = data.PWM.(part);
            motor.pwm = temp_pwm(:,number);
            motor.voltage = motor.ratioVoltage*motor.pwm;
            motor.torqueMachine = data.tau*motor.ratioTorque;
            if isfield(data,'current')
                motor.current = data.current;
            end
            motor.time = data.time;
            if size(motor.friction,1) > 0
                motor = motor.evaluateCoefficient();
            end
        end
        
        function plotCollected(motor, typedata)
            %% Plot Collected images - Motor and model
            hold on
            motor.plotMotor(typedata);
            motor.plotMotorModel(typedata);
            hold off
        end
        
        function plotMotorModel(motor, typedata)
            %% Plot Motor model
            if strcmp(typedata,'Firmware')
                unitX = 'PWM';
                unitY = 'Torque Machine';
                Ktvalue = motor.KtFirmware;
            else
                unitX = 'V';
                unitY = 'Nm';
                Ktvalue = motor.Kt;
            end
            title(sprintf('Kt: %12.8f [%s]/[%s] - Name motor: %s',Ktvalue, unitY, unitX, motor.name));
            
            if strcmp(typedata,'Firmware')
                plot(motor.pwm, motor.pwm*motor.KtFirmware,'r-','LineWidth',3);
            else
                plot(motor.voltage, motor.voltage*motor.Kt,'r-','LineWidth',3);
            end
        end
        
        function plotMotor(motor, typedata, option)
            %% Plot measure versus friction estimation
            if ~exist('option','var')
                option = '.';
            end
            if strcmp(typedata,'Firmware')
                plot(motor.pwm, motor.ratioTorque*(motor.torque-motor.friction_model), option);
                xlabel('PWM','Interpreter','tex');
                ylabel('TorqueMachine','Interpreter','tex');
            else
                plot(motor.voltage, motor.torque-motor.friction_model, option);
                xlabel('Voltage','Interpreter','tex');
                ylabel('\tau-\tau_{f}','Interpreter','tex');
            end
        end
        
        function motor = evaluateCoefficient(motor)
            %% Evaluate coefficient for Kt motor
            motor.friction_model = motor.friction.getFriction(motor.qdot);
            A = Joint.linearRegression(motor.pwm,motor.torqueMachine-motor.friction_model);
            B = Joint.linearRegression(motor.voltage,motor.torque-motor.friction_model);
            %A = joint.linearRegression(joint.current,joint.torque-joint.friction_model);
            motor.KtFirmware = A(1);
            motor.Kt = B(1);
        end

        function text = saveCoeffToFile(motor, typedata)
            %% Write information to txt file
            text = '';
            if strcmp(typedata,'Firmware')
                if size(motor.KtFirmware,1) ~= 0
                    text = [text, sprintf('\n---------->     Kt - FIRMWARE    <----------\n')];
                    text = [text, sprintf('tau_machine = Kt*PWM \n')];
                    unitX = 'PWM';
                    unitY = 'TorqueMachine';
                    text = [text, sprintf('Kt: %12.8f [%s]/[%s]',motor.KtFirmware, unitY, unitX)];
                    text = [text, sprintf('\nRatio Volt: %12.8f [V]/PWM',motor.ratioVoltage)];
                    text = [text, sprintf('\nRatio Torque: %12.8f [Nm]/TorqueMachine',motor.ratioTorque)];
                end
            else
                if size(motor.Kt,1) ~= 0
                    text = [text, sprintf('\n---------->     Kt    <----------\n')];
                    text = [text, sprintf('tau_m = Kt*Volt \n')];
                    unitX = 'V';
                    unitY = 'Nm';
                    text = [text, sprintf('Kt: %12.8f [%s]/[%s]',motor.Kt, unitY, unitX)];
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
        
        function motor = controlValue(motor,typedata)
            %% Control values
            if strcmp(typedata,'Firmware')
                if length(motor.Kt) == 1
                    % motor.bemf = [Nm]/([deg]/[s])
                    % motor.Kt = [Nm]/[V] -- 1/motor.Kt = [V]/[Nm]
                    % motor.ratioTorque = [TorqueMachine]/[Nm]
                    % motor.ratioVoltage = [V]/[PWM]
                    motor.ktau = (1/(motor.ratioVoltage*motor.Kt)); % [PWM]/[Nm]
                    motor.stictionUp = sign(motor.ktau)*abs(motor.friction.KcP); % [Nm]
                    motor.stictionDown = sign(motor.ktau)*abs(motor.friction.KcN); % [Nm]
                    motor.bemf = sign(motor.ktau)*mean([abs(motor.friction.KvP),abs(motor.friction.KvN)]);
                end
            elseif strcmp(typedata,'Firmware-CAN')
                if length(motor.KtFirmware) == 1
                    % motor.bemf = [Nm]/([deg]/[s])
                    % motor.KtFirmware = [Nm]/[TorqueMachine] -- 1/motor.Kt = [TorqueMachine]/[Nm]
                    % motor.ratioTorque = [TorqueMachine]/[Nm]
                    % motor.ratioVoltage = [V]/[PWM]
                    motor.ktau = (1/(motor.KtFirmware)); % [TorqueMachine]/[Nm]
                    motor.stictionUp = sign(motor.ktau)*abs(motor.friction.KcP*motor.ratioTorque); 
                    motor.stictionDown = sign(motor.ktau)*abs(motor.friction.KcN*motor.ratioTorque);
                    motor.bemf = sign(motor.ktau)*mean([abs(motor.friction.KvP*motor.ratioTorque),abs(motor.friction.KvN*motor.ratioTorque)]);
                end
            else
                if length(motor.Kt) == 1
                    motor.ktau = 1/motor.Kt;
                    % motor.ratioTorque = [TorqueMachine]/[Nm]
                    % motor.ratioVoltage = [V]/[PWM]
                    motor.stictionUp = sign(motor.ktau)*abs(motor.friction.KcP*motor.ktau);
                    motor.stictionDown = sign(motor.ktau)*abs(motor.friction.KcN*motor.ktau);
                    motor.bemf = sign(motor.ktau)*abs(mean([motor.friction.KvP*motor.ktau,motor.friction.KvN*motor.ktau]));
                end
            end
        end
        
        function text = textControlData(motor, typedata)
            %% Information joint estimation
            motor = motor.controlValue(typedata);
            if strcmp(typedata,'Firmware')
                if size(motor.KtFirmware,1) ~= 0
                    unitX = 'PWM';
                    unitY = 'Nm';
                    text = sprintf('\n---------->  Parameters for FIRMWARE  <----------\n\n');
                    text = [text sprintf('ktau:\t\t\t%12.8f [%s]/[%s]\n',motor.ktau,unitX,unitY)];
                    text = [text sprintf('stictionUp:\t\t%12.8f [%s]\n', motor.stictionUp,unitY)];
                    text = [text sprintf('stictionDown:\t%12.8f [%s]\n', motor.stictionDown,unitY)];
                    text = [text sprintf('bemf:\t\t\t%12.8f [%s][s]/[deg]\n', motor.bemf,unitY)];
                    %fprintf(fileID,'KsP: %12.8f [Nm] - KsN %12.8f [Nm][s]/[deg]\n',joint.friction.KsP, joint.friction.KsN);
                end
            else
                if size(motor.Kt,1) ~= 0
                    unitX = 'V';
                    unitY = 'Nm';
                    text = sprintf('\n---------->  Parameters for joint torque control  <----------\n\n');
                    text = [text sprintf('ktau:\t\t\t%12.8f [%s]/[%s]\n',motor.ktau,unitX,unitY)];
                    text = [text sprintf('stictionUp:\t\t%12.8f [%s]\n', motor.stictionUp,unitY)];
                    text = [text sprintf('stictionDown:\t%12.8f [%s]\n', motor.stictionDown,unitY)];
                    text = [text sprintf('bemf:\t\t\t%12.8f [%s][s]/[deg]\n', motor.bemf,unitY)];
                    %fprintf(fileID,'KsP: %12.8f [Nm] - KsN %12.8f [Nm][s]/[deg]\n',joint.friction.KsP, joint.friction.KsN);
                end
            end
            
        end
    end
    
end

