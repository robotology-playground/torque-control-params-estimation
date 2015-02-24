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
            for i=1:size(list_motors,1)
                part_list = regexp(list_motors{i}, '\,', 'split');
                n_matrix = regexp(part_list{1}, '\w*\.\w*', 'match');
                [~,name_motor] = regexp(part_list{2}, '\"(\w*).*?\"', 'match','tokens');
                new_motor = Motor(name_motor, str2double(n_matrix{1}));
                new_motor = new_motor.loadParameter(path_project);
                joint.motor = [joint.motor new_motor];
            end
        end
        
        function joint = loadData(joint, type, data)
            %% Load Friction measure
            if strcmp(type,'ref')
                joint.motor = joint.motor.loadMeasure(data, joint.part, joint.number);
            elseif strcmp(type,'idle')
                joint.motor.friction = Friction(data.q, data.qD, data.qDD, data.tau, data.time);
            end
        end
        
        function number = asPlot(joint)
            %% Get Number of subplot
            if size(joint.motor.friction,1) > 0
                if size(joint.motor.Kt,1) > 0
                    number = 2;
                else
                    number = 1;
                end
            else
                number = 0;
            end
        end
        
        function text = saveToFile(joint)
            %% Save All information joint to File
            text = joint.getInformation();
            text = [text sprintf('\n')];
            text = [text joint.motor.friction.saveToFile()];
            if size(joint.motor.Kt,1) > 0
                text = [text sprintf('\n')];
                text = [text joint.motor.saveCoeffToFile()];
                text = [text sprintf('\n')];
                text = [text joint.motor.textControlData()];
            end
        end
        
        function text = saveLatexToFile(joint)
            %% Save All information in latex format to file
            text = joint.getInformation();
            text = [text sprintf('\n')];
            text = [text joint.motor.friction.saveLatexToFile(joint.name)];
            if size(joint.motor.Kt,1) > 0
                text = [text sprintf('\n')];
                text = [text joint.motor.saveLatexCoeffToFile()];
%                 text = [text sprintf('\n')];
%                 text = [text joint.motor.textControlData()];
            end
        end
        
        function plotCollected(joint)
            %% Plot Data
            if size(joint.motor.friction,1) > 0
                if size(joint.motor.Kt,1) > 0
                    subplot(1,2,1);
                    joint.motor.friction.plotCollected();
                    grid;
                    subplot(1,2,2);
                    joint.motor.plotCollected();
                    grid;
                else
                    joint.motor.friction.plotCollected();
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

