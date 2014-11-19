classdef Friction
    %FRICTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess=private,GetAccess=private)
        position;
        velocity;
        torque;
        time;
        th_velocity;
        step;
    end
    
    properties
        AP = [];
        AN = [];
    end
    
    methods
        function obj = Friction(position,velocity,torque,time,th_velocity)
            obj.position = position;
            obj.velocity = velocity;
            obj.torque = torque;
            obj.time = time;
            obj.step = time(2);
            if ~exist('th_velocity','var')
                obj.th_velocity = 0.1;
            else
                obj.th_velocity = th_velocity;
            end
            % Evaluate fricition
            obj.AP = obj.linearRegression(velocity(velocity > obj.th_velocity), ...
                                                torque(velocity > obj.th_velocity));
            obj.AN = obj.linearRegression(velocity(velocity < -obj.th_velocity), ...
                                                torque(velocity < -obj.th_velocity));
        end
        
        function coeff = getFrictionData(obj, th_velocity, option)
            if ~exist('th_velocity','var')
                th_velocity = 1;
            end
            % Evaluate fricition
            coeff = struct;
            coeff.AP = obj.linearRegression(obj.velocity(obj.velocity > th_velocity), ...
                                                obj.torque(obj.velocity > th_velocity));
            coeff.AN = obj.linearRegression(obj.velocity(obj.velocity < -th_velocity), ...
                                                obj.torque(obj.velocity < -th_velocity));
            if ~exist('option','var')
                option = 'r';
            end
            x = 0:obj.step:max(obj.velocity);
            hold on;
            plot(x, x*coeff.AP(1)+coeff.AP(2),option,'LineWidth',3);
            x = min(obj.velocity):obj.step:0;
            plot(x, x*coeff.AN(1)+coeff.AN(2),option,'LineWidth',3);
            
            plot([0 0], [coeff.AP(2) coeff.AN(2)] ,option,'LineWidth',3);
            
            hold off;
        end
        
        function plotTree(obj, coeff,option)
            if ~exist('option','var')
                option = '-';
            end
            
            subplot(1,2,1);
            hold on;
            x = 0:obj.step:max(obj.velocity);
            plot(x, coeff.AP(2)*ones(size(x,2),1),option);
            
            x = min(obj.velocity):obj.step:0;
            plot(x, coeff.AN(2)*ones(size(x,2),1),option);
            
            plot([0 0], [coeff.AP(2) coeff.AN(2)] ,option);
            grid;
            hold off;
            
            subplot(1,2,2);
            hold on;
            x = 0:obj.step:max(obj.velocity);
            plot(x, x*coeff.AP(1),option);
            x = min(obj.velocity):obj.step:0;
            plot(x, x*coeff.AN(1),option);
            grid;
            hold off;
        end
        
        function plotFriction(obj, option)
            if ~exist('option','var')
                option = '.';
            end
            plot(obj.velocity, obj.torque, option);
            title('Relation between torque, velocity');
            xlabel('qdot','Interpreter','tex');
            ylabel('\tau','Interpreter','tex');
        end
        
        function obj = setThVelocity(obj, th_velocity)
            obj.th_velocity = th_velocity;
        end
        
        function th = getThVelocity(obj)
            th = obj.th_velocity;
        end
    end
    
    methods (Access = protected, Static)
        function a = linearRegression(x, y)
            N = size(x,1);
            % Add column of 1's to include constant term in regression
            X = [x ones(N,1)]; 
            % = [a1; a0]
            a = regress(y,X);
        end
    end
    
end

