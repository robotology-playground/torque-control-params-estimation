classdef CoupledJoints
    %COUPLEDJOINT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    pitch;
    roll;
    yaw;
    T;
    end
    
    methods
        function coupled = CoupledJoints(start_folder, robotName, part, type)
                coupled.pitch = Motor(start_folder, robotName, part, type,'pitch');
                coupled.roll = Motor(start_folder, robotName, part, type,'roll');
                coupled.yaw = Motor(start_folder, robotName, part, type,'yaw');
        end
        
        function coupled = addTransformMatrix(T)
            coupled.T = T;
        end
    end
    
end

