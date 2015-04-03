classdef Friction
    %FRICTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(SetAccess=private, GetAccess=private)
        position;
        velocity;
        acceleration;
        torque;
        time;
        step;
        noise;
        experiment;
    end
    
    properties
        th_velocity;
        KcP;
        KcN;
        KvP;
        KvN;
        KsP;
        KsN;
        offset = 0;
        cutoff;
    end
    
    methods
        function obj = Friction(position, velocity, acceleration, torque, time, cutoff, th_velocity)
            obj.position = position;
            obj.velocity = velocity;
            if size(acceleration,1) == 0
                obj.acceleration = zeros(size(obj.velocity,1),1);
            else
                obj.acceleration = acceleration;
            end
            obj.torque = torque;
            obj.time = time;
            obj.step = time(2)-time(1);
            
            if ~exist('cutoff','var')
                %obj.cutoff = 1/obj.step;
                obj.cutoff = 0;
            else
                obj.cutoff = cutoff;
            end
            
%             if exist('th_velocity','var')
%                 obj = obj.evaluateCoeff(th_velocity);
%             else
%                 obj = obj.minTh_vel();
%             end
            obj = obj.evaluateCoeff(500);
            obj.experiment = '';
        end

        function obj = minTh_vel(obj, step, velMax)
            if ~exist('step','var')
                step = 0.1;
            end
            if ~exist('velMax','var')
                velMax = max(obj.velocity)/2;
            end
            obj = obj.evaluateCoeff(0);
            Var = zeros(floor(velMax/step),1);
            for i=1:size(Var,1);
                obj = obj.evaluateCoeff(10+i*step);
                variance = var(obj.torque-obj.getFriction(obj.velocity));
                Var(i) = variance;
            end
            [~, i] = min(Var);
            obj.th_velocity = 10+i*step;
            
            obj = obj.evaluateCoeff(obj.th_velocity);
        end
        
        function obj = evaluateCoeff(obj, th_velocity)
            %% Evaluate Coefficent
            if ~exist('th_velocity','var')
                obj.th_velocity = 1;
            else
                obj.th_velocity = th_velocity;
            end
            
            if obj.cutoff ~= 0
                fc = obj.cutoff;
                Fs = size(obj.time,1);
                Wn = (2/Fs)*fc;
                b = fir1(20,Wn,'low',kaiser(21,3));
                obj.torque = filter(b,1,obj.torque);
                obj.velocity = filter(b,1,obj.velocity);
            end
            
            % Evaluate fricition
            AP = Joint.linearRegression(obj.velocity(obj.velocity > th_velocity/2), ...
                obj.torque(obj.velocity > th_velocity/2));
            AN = Joint.linearRegression(obj.velocity(obj.velocity < -th_velocity/2), ...
                obj.torque(obj.velocity < -th_velocity/2));

%             AP = Joint.linearRegression(obj.velocity((obj.velocity > th_velocity/2) & (obj.acceleration <= 0)), ...
%                 obj.torque((obj.velocity > th_velocity/2) & (obj.acceleration <= 0)));
%             AN = Joint.linearRegression(obj.velocity((obj.velocity < -th_velocity/2) & (obj.acceleration >= 0)), ...
%                 obj.torque((obj.velocity < -th_velocity/2) & (obj.acceleration >= 0)));
            
            obj.KcP = AP(2);
            obj.KvP = AP(1);
            obj.KcN = AN(2);
            obj.KvN = AN(1);
            
            % if obj.cutoff ~= 0
            %     obj.noise = filter(b,1, obj.torque-obj.getFriction(obj.velocity));
            % else
            obj.noise = obj.torque-obj.getFriction(obj.velocity);
            % end
            obj.KsP = max(obj.noise);
            obj.KsN = min(obj.noise);
            
        end
        
        function obj = setToCenter(obj)
            %% Remove from data offset from wrong models
            %obj.offset = mean(obj.torque);
            obj.offset = (obj.KcP-obj.KcN)/2 + obj.KcN;
            %Scaled Kc and Kv
            obj.torque = obj.torque-obj.offset;
            obj.KcP = obj.KcP - obj.offset;
            obj.KcN = obj.KcN - obj.offset;
        end
        
        function friction = getFriction(obj, qdot)
            %% Evaluate friction model
            friction = zeros(size(qdot,1),1);
%             friction(qdot < -obj.th_velocity/2) = obj.KcN + obj.KvN*qdot(qdot < -obj.th_velocity/2);
%             friction(qdot > obj.th_velocity/2) = obj.KcP + obj.KvP*qdot(qdot > obj.th_velocity/2);
%             friction(qdot >= -obj.th_velocity/2 & qdot <= obj.th_velocity/2) = (obj.KcP-obj.KcN)/2 + obj.KcN;
            %friction(qdot >= -obj.th_velocity/2 & qdot <= obj.th_velocity/2) = obj.torque(qdot >= -obj.th_velocity/2 & qdot <= obj.th_velocity/2);
            friction(qdot > 0) = obj.KcP + obj.KvP*qdot(qdot > 0);
            friction(qdot == 0) = (obj.KcP-obj.KcN)/2 + obj.KcN;
            friction(qdot < 0) = obj.KcN + obj.KvN*qdot(qdot < 0);
        end
        
        function plotFrictionModel(obj, option)
            %% Plot Friction Model
            if ~exist('option','var')
                option = 'r';
            end
            
            hold on;
            x = 0:obj.step:max(obj.velocity);
            plot(x, x*obj.KvP + obj.KcP,option,'LineWidth',3);
            x = min(obj.velocity):obj.step:0;
            plot(x, x*obj.KvN + obj.KcN,option,'LineWidth',3);
            plot([0 0], [obj.KcP obj.KcN] ,option,'LineWidth',3);
            %plot([0 0], [obj.KsP obj.KsN] ,option,'LineWidth',3);
            hold off;
        end
        
        function text = saveToFile(obj)
            %% Save information about friction on file
            text = sprintf('\n----------> Friction <----------\n');
            % Coefficients
            text = [text, sprintf('tau_f = s(q)(BAR-KcP + BAR-KvP*qdot) + s(-q)(BAR-KcN + BAR-KvN*qdot)\n\n')];
            text = [text, sprintf('BAR-KcP: %12.8f [Nm] - BAR-KcN: %12.8f [Nm]\n',obj.KcP, obj.KcN)];
            text = [text, sprintf('BAR-KvP: %12.8f [Nm][s]/[deg] - BAR-KvN: %12.8f [Nm][s]/[deg]',obj.KvP, obj.KvN)];
            %text = [text, sprintf('KsP: %12.8f [Nm] - KsN %12.8f [Nm][s]/[deg]\n',obj.KsP, obj.KsN)];
        end
        
        function text = saveLatexToFile(obj, path)
            %% Save information about friction on file
            text = sprintf('\n---- Friction -> Latex ----\n');
            % To latex
            text = [text, sprintf('\\begin{equation}\n')];
            text = [text, sprintf('\\label{eq:Friction}\n')];
            text = [text, sprintf('\\begin{array}{ccc}\n')];
            text = [text, sprintf('\\tau_f & = & s(q)\\left(\\bar K_{c+} + \\bar K_{v+}\\dot q\\right) + s(-q)\\left(\\bar K_{c-} + \\bar K_{v-}*\\dot q\\right)\n')];
            text = [text, sprintf('\\end{array}\n')];
            text = [text, sprintf('\\end{equation}\n')];
            
            text = [text, sprintf('\\begin{equation}\n')];
            text = [text, sprintf('\\label{eq:%sFrictionCoeffCoulomb}\n',path)];
            text = [text, sprintf('\\begin{array}{cccl}\n')];
            text = [text, sprintf('\\bar K_{c+} & \\simeq & %12.8f & [Nm] %s\n',obj.KcP,'\\')];
            text = [text, sprintf('\\bar K_{c-} & \\simeq & %12.8f & [Nm]\n',obj.KcN)];
            text = [text, sprintf('\\end{array}\n')];
            text = [text, sprintf('\\end{equation}\n')];
            
            text = [text, sprintf('\n\\begin{equation}\n')];
            text = [text, sprintf('\\label{eq:%sFrictionCoeffViscous}\n',path)];
            text = [text, sprintf('\\begin{array}{cccl}\n')];
            text = [text, sprintf('\\bar K_{v+} & \\simeq & %12.8f & \\frac{[Nm][s]}{[deg]} %s\n',obj.KvP,'\\')];
            text = [text, sprintf('\\bar K_{v-} & \\simeq & %12.8f & \\frac{[Nm][s]}{[deg]}\n',obj.KvN)];
            text = [text, sprintf('\\end{array}\n')];
            text = [text, sprintf('\\end{equation}\n')];
            
%             text = [text, sprintf('\n\\begin{equation}\n')];
%             text = [text, sprintf('\\label{eq:%sFrictionCoeffStiction}\n',path)];
%             text = [text, sprintf('\\begin{array}{cccl}\n')];
%             text = [text, sprintf('\\bar K_{s+} & \\simeq & %12.8f & [Nm] %s\n',obj.KsP,'\\')];
%             text = [text, sprintf('\\bar K_{s-} & \\simeq & %12.8f & [Nm]\n',obj.KsN)];
%             text = [text, sprintf('\\end{array}\n')];
%             text = [text, sprintf('\\end{equation}\n')];
        end
        
        
        function plotNoise(obj, option)
            %% Plot noise
            if ~exist('option','var')
                option = '.';
            end
            plot(obj.velocity,obj.noise,option);
            xlabel('qdot','Interpreter','tex');
            ylabel('\tau','Interpreter','tex');
        end
        
        function plotParts(obj, option)
            %% Plot Parts
            if ~exist('option','var')
                option = '-';
            end
            
            subplot(1,2,1);
            hold on;
            x = 0:obj.step:max(obj.velocity);
            plot(x, obj.KcP*ones(size(x,2),1),option);
            
            x = min(obj.velocity):obj.step:0;
            plot(x, obj.KcN*ones(size(x,2),1),option);
            
            plot([0 0], [obj.KcP obj.KcN] ,option);
            grid;
            hold off;
            xlabel('qdot','Interpreter','tex');
            ylabel('\tau','Interpreter','tex');
            
            subplot(1,2,2);
            hold on;
            x = 0:obj.step:max(obj.velocity);
            plot(x, x*obj.KvP,option);
            x = min(obj.velocity):obj.step:0;
            plot(x, x*obj.KvN,option);
            grid;
            hold off;
            xlabel('qdot','Interpreter','tex');
            ylabel('\tau','Interpreter','tex');
        end

        function plotFriction(obj, option)
            %% Plot friction
            if ~exist('option','var')
                option = '.';
            end
%             qD_d = obj.velocity((obj.velocity > obj.th_velocity/2) & (obj.acceleration >= 0));
%             qD_d = [qD_d; obj.velocity((obj.velocity < -obj.th_velocity/2) & (obj.acceleration <= 0))];
%             fr = obj.torque((obj.velocity > obj.th_velocity/2) & (obj.acceleration >= 0));
%             fr = [fr; obj.torque((obj.velocity < -obj.th_velocity/2) & (obj.acceleration <= 0))];
            
            plot(obj.velocity,obj.torque, option);
%             hold on;
%             plot(qD_d, fr,'g.');
%             hold off;
            xlabel('qdot [deg]/[s] (Velocity)','Interpreter','tex');
            ylabel('\tau [Nm] (Torque)','Interpreter','tex');
        end
        
        function plotCollected(obj)
            hold on;
            obj.plotFriction();
            obj.plotFrictionModel();
            hold off;
            title('Friction estimate');
        end
        
        function obj = setExperiment(obj,experiment)
            obj.experiment = experiment;
        end
        
        %% Get original data: Velocity and torque
        function data = getInfo(obj)
            data = struct;
            data.velocity = obj.velocity;
            data.torque = obj.torque;
        end
    end
    
end

