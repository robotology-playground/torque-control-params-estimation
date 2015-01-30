classdef Friction
    %FRICTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(SetAccess=private, GetAccess=private)
        position;
        velocity;
        torque;
        time;
        step;
    end
    
    properties
        th_velocity;
        KcP;
        KcN;
        KvP;
        KvN;
        offset = 0;
    end
    
    methods
        function obj = Friction(position,velocity,torque,time,th_velocity)
            obj.position = position;
            obj.velocity = velocity;
            obj.torque = torque;
            obj.time = time;
            obj.step = time(2)-time(1);

            obj = obj.evaluateCoeff(th_velocity);

        end
        
        %% Evaluate Coefficent
        function obj = evaluateCoeff(obj, th_velocity)
            
            if ~exist('th_velocity','var')
                obj.th_velocity = 1;
            else
                obj.th_velocity = th_velocity;
            end
            
            % Evaluate fricition
            AP = obj.linearRegression(obj.velocity(obj.velocity > th_velocity), ...
                                                obj.torque(obj.velocity > th_velocity));
            AN = obj.linearRegression(obj.velocity(obj.velocity < -th_velocity), ...
                                                obj.torque(obj.velocity < -th_velocity));
                                            
            obj.KcP = AP(2);
            obj.KvP = AP(1);
            obj.KcN = AN(2);
            obj.KvN = AN(1);
        end
        
        %% Remove from data offset from wrong models
        function obj = setToCenter(obj)
            if obj.KcP < 0
                obj.torque = -obj.torque;
                obj.KcP = -obj.KcP;
                obj.KcN = -obj.KcN;
                obj.KvP = -obj.KvP;
                obj.KvN = -obj.KvN;
            end
            obj.offset = mean(obj.torque);
            %Scaled Kc and Kv
            obj.offset = obj.KcP - obj.KcN;
            obj.torque = obj.torque-(obj.KcP-obj.offset/2);
            obj.KcP = obj.offset/2;
            obj.KcN = -obj.offset/2;
        end
        
        %% Evaluate friction model
        function friction = getFriction(obj, qdot)
            friction = zeros(size(qdot,1),1);
            friction(qdot < -obj.th_velocity) = obj.KcN + obj.KvN*qdot(qdot < -obj.th_velocity);
            friction(qdot > obj.th_velocity) = obj.KcP + obj.KvP*qdot(qdot > obj.th_velocity);
        end
        
        %% Plot Friction Model
        function plotFrictionModel(obj, option)
            if ~exist('option','var')
                option = 'r';
            end
            
            hold on;
            x = 0:obj.step:max(obj.velocity);
            plot(x, x*obj.KvP + obj.KcP,option,'LineWidth',3);
            x = min(obj.velocity):obj.step:0;
            plot(x, x*obj.KvN + obj.KcN,option,'LineWidth',3);
            plot([0 0], [obj.KcP obj.KcN] ,option,'LineWidth',3);
            hold off;
        end
        
        %% Plot noise
        function plotNoise(obj, option)
            if ~exist('option','var')
                option = '.';
            end
            plot(obj.velocity,obj.torque-obj.getFriction(obj.velocity),option);
            xlabel('qdot','Interpreter','tex');
            ylabel('\tau','Interpreter','tex');
        end
        
        %% Plot Parts
        function plotParts(obj, option)
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
        
        %% Plot friction
        function plotFriction(obj, option)
            if ~exist('option','var')
                option = '.';
            end
            plot(obj.velocity, obj.torque, option);
            title('Relation between torque, velocity');
            xlabel('qdot','Interpreter','tex');
            ylabel('\tau','Interpreter','tex');
        end
        
        %% Get original data: Velocity and torque
        function data = getInfo(obj)
            data = struct;
            data.velocity = obj.velocity;
            data.torque = obj.torque;
        end
    end
    
    methods (Access = protected, Static)
        %% Linear regression to evalute coefficent for friction
        function a = linearRegression(x, y)
            N = size(x,1);
            % Add column of 1's to include constant term in regression
            X = [x ones(N,1)]; 
            % = [a1; a0]
            a = regress(y,X);
        end
    end
    
end

