classdef CoupledJoints
    %COUPLEDJOINT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = protected)
        pitch;
        roll;
        yaw;
        T;
    end
    
    properties
        robot;
        path;
    end
    
    methods
        function coupled = CoupledJoints(start_folder, robotName, part, type)
            if ~exist('type','var')
                coupled.pitch = Motor(start_folder, robotName, part, 'pitch');
                coupled.roll = Motor(start_folder, robotName, part, 'roll');
                coupled.yaw = Motor(start_folder, robotName, part, 'yaw');
            else
                coupled.pitch = Motor(start_folder, robotName, part, type, 'pitch');
                coupled.roll = Motor(start_folder, robotName, part, type, 'roll');
                coupled.yaw = Motor(start_folder, robotName, part, type, 'yaw');
            end
            if strcmp(part,'torso')
                R = 0.04;
                r = 0.022;
                coupled.T = [r/R r/(2*R) r/(2*R);
                    0    1/2     1/2;
                    0   -1/2     1/2];
            elseif strcmp(part,'arm')
                t = 0.625;
                if strcmp(type,'left')
                    coupled.T = [-1     0	0;
                        -1    -t	0;
                        0     t  -t];
                elseif strcmp(type,'right')
                    coupled.T = [1 0 0;
                        1 t 0;
                        0 -t t];
                end
            end
            coupled.path = coupled.pitch.getPathType();
            coupled.robot = robotName;
        end
        
        function coupled = changeTmatrix(coupled, T)
            coupled.T = T;
        end
        
        function coupled = loadIdleMeasure(coupled, file, threshold, cutoff)
            
        end
    end
end

