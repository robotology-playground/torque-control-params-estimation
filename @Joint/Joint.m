classdef Joint
    %JOINT Summary of this class goes here
    %   Detailed explanation goes here

    properties (Access = private)
    end
    
    properties
        name;
        number;
        part;
        motor;
    end
    
    methods
        function joint = Joint(name, part, number)
            joint.name = name;
            joint.number = str2double(number) + 1;
            joint.part = part;
            joint.motor = Motor('untitled',0);
        end
        
        function joint = setMotor(joint, path_project, list_motors)
            %% Set motor in joints
            joint.motor = [];
            for i=1:size(list_motors,2)
                part_list = regexp(list_motors{i}, '\,', 'split');
                n_matrix = regexp(part_list{1}, '\w*\.\w*', 'match');
                [~,name_motor] = regexp(part_list{2}, '\"(\w*).*?\"', 'match','tokens');
                new_motor = Motor(name_motor{1}{1}, str2double(n_matrix{1}));
                new_motor = new_motor.loadParameter(path_project);
                joint.motor = [joint.motor new_motor];
            end
        end
        
        function joint = loadData(joint, type, data, motor_name)
            %% Load Friction measure
            if exist('motor_name','var')
                idx = joint.getIndexMotorFromList(motor_name);
            else
                idx = 1;
            end
            if idx ~= 0
                if strcmp(type,'ref')
                    joint.motor(idx) = joint.motor(idx).loadMeasure(data, joint.part, joint.number);
                elseif strcmp(type,'idle')
                    joint.motor(idx).friction = Friction(data.q, data.qD, data.qDD, data.tau, data.time);
                end
            end
        end
        
        function index = getIndexMotorFromList(joint, motor_name)
            %% Get index motor from name
            if size(joint.motor,2) > 0
                for i=1:size(joint.motor,2)
                    if strcmp(joint.motor(i).name,motor_name)
                        index = i;
                        return
                    end
                end
            end
            index = 0;
        end
        
        function number = asPlot(joint)
            %% Get Number of subplot
            if size(joint.motor(1).friction,1) > 0
                if size(joint.motor(1).getKt(),1) > 0
                    number = 2;
                else
                    number = 1;
                end
            else
                number = 0;
            end
        end
        
        function text = saveVerbose(joint, idx_motor)
            %% Save All information joint to File
            text = sprintf('================================\n');
            text = [text 'Motor: ' joint.motor(idx_motor).name sprintf('\n')];
            text = [text joint.motor(idx_motor).friction.saveToFile()];
            if size(joint.motor(idx_motor).getKt(),1) > 0
                text = [text sprintf('\n')];
                text = [text joint.motor(idx_motor).saveCoeffToFile()];
                text = [text sprintf('\n')];
                text = [text joint.motor(idx_motor).textControlData()];
            end
        end
        
        function text = saveToFile(joint, typedata, idx_motor)
            %% Save All information joint to File
            text = sprintf('================================\n');
            text = [text 'Motor: ' joint.motor(idx_motor).name ' (' joint.name sprintf(')\n')];
            if strcmp(typedata,'Firmware')
                if size(joint.motor(idx_motor).getKtFirmware(),1) > 0
                    text = [text joint.motor(idx_motor).textControlData(typedata)];
                end
            else
                if size(joint.motor(idx_motor).getKt(),1) > 0
                    text = [text joint.motor(idx_motor).textControlData(typedata)];
                end
            end
        end
        
        function text = saveLatexToFile(joint)
            %% Save All information in latex format to file
            text = joint.getInformation();
            text = [text sprintf('\n')];
            text = [text joint.motor.friction.saveLatexToFile(joint.name)];
            if size(joint.motor.getKt(),1) > 0
                text = [text sprintf('\n')];
                text = [text joint.motor.saveLatexCoeffToFile()];
%                 text = [text sprintf('\n')];
%                 text = [text joint.motor.textControlData()];
            end
        end
        
        function plotCollected(joint, typedata, counter)
            if ~exist('counter','var')
                counter = 1;
            end
            %% Plot Data
            if size(joint.motor(counter).friction,1) > 0
                if size(joint.motor(counter).getKt(),1) > 0
                    subplot(1,2,1);
                    joint.motor(counter).friction.plotCollected();
                    grid;
                    subplot(1,2,2);
                    joint.motor(counter).plotCollected(typedata);
                    grid;
                else
                    joint.motor(counter).friction.plotCollected();
                    grid;
                end
            end
        end
        
        function text = getInformation(joint)
            text = sprintf('Name Joint: %s\n',joint.name);
            text = [text sprintf('Part: %s\n',joint.part)];
            text = [text sprintf('Number: %d',joint.number-1)];
        end
        
        %function plot
    end
    
    methods (Access = public, Static)
        function a = linearRegression(x, y)
            %% Linear regression to evalute coefficent for friction
            % Line equal y = a(1)*x + a(2)
            a = zeros(2,1);
            r = corrcoef(x,y); % Corr coeff is the off-diagonal (1,2) element
            r = r(1,2);  % Sample regression coefficient
            xbar = mean(x);
            ybar = mean(y);
            sigx = std(x);
            sigy = std(y);
            a1 = r*sigy/sigx;   % Regression line slope
            %yfit = ybar - a1*xbar + a1*x;
            a(1) = a1;
            a(2) = ybar - a1*xbar;
        end
    end
    
end

