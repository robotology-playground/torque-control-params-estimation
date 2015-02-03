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
            coupled.pitch = coupled.pitch.setPart('number_joint',1);
            coupled.roll = coupled.roll.setPart('number_joint',1);
            coupled.yaw = coupled.yaw.setPart('number_joint',1);
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
            %% Load data from mat file
            if ~exist('file','var')
                file = 'idle';
            end
            if ~exist('threshold','var')
                threshold = 1;
            end
            data = load([coupled.path file '.mat']);
            
            coupled.pitch.loadIdleMeasureData(data.logsout.get('q').Values.Data(:,numb), ...
                data.logsout.get('qD').Values.Data(:,numb), ...
                data.logsout.get('tau').Values.Data(:,numb), ...
                data.time, threshold);
        end
        
        function command = getControlBoardCommand(joint, rate)
            %% Get string to start ControlBoardDumper
            if ~exist('rate','var')
                rate = 10;
            end
            command = ['controlBoardDumper'...
                ' --robot icub'...
                ' --part ' joint.pitch.group_select ...
                ' --rate ' num2str(rate) ...
                ' --joints "(0 1 2)"' ...
                ' --dataToDump "(getOutputs getCurrents)"'];
        end
    end
end

