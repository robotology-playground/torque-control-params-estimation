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
            joint.number = str2double(number);
            joint.part = part;
        end
        
        function joint = setMotor(joint, name_motor, Voltage, range_pwm)
            %% Set motor in joints
            joint.motor = Motor(name_motor, Voltage, range_pwm);
        end
%         saveParameters(obj);
%         obj = loadParameters(obj, file);
%         obj = loadIdleMeasure(obj, file, cutoff);
%         obj = loadRefMeasure(obj, file);
%         obj = setRatio(obj, Voltage, range_pwm);
%         list = getJointList(obj);
%         [hFig, counter] = savePictureFriction(obj, counter);
%         [hFig, counter] = savePictureKt(obj, counter);
%         saveToFile(obj, name);
%         saveControlToFile(obj, name);
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

