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
            % Wiki reference
            % http://wiki.icub.org/wiki/ICub_coupled_joints
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
            
            q_data = data.logsout.get('q').Values.Data;
            qdot_data = data.logsout.get('qD').Values.Data;
            %qddot_data = data.logsout.get('qDD').Values.Data;
            tau_data = data.logsout.get('tau').Values.Data;
            
            m     = (coupled.T^-1*q_data')';
            md    = (coupled.T^-1*qdot_data')';
            %mdd    = (coupled.T^-1*qddot_data')';
            tau_m = (coupled.T'*tau_data')';

            numb_pitch = coupled.pitch.number_part;
            numb_roll = coupled.roll.number_part;
            numb_yaw = coupled.yaw.number_part;
            if ~exist('cutoff','var')
                coupled.pitch = coupled.pitch.loadIdleMeasureData(m(:,numb_pitch), ...
                    md(:,numb_pitch), ...
                    tau_m(:,numb_pitch), ...
                    data.time, threshold);
                coupled.roll = coupled.roll.loadIdleMeasureData(m(:,numb_roll), ...
                    md(:,numb_roll), ...
                    tau_m(:,numb_roll), ...
                    data.time, threshold);
                coupled.yaw = coupled.yaw.loadIdleMeasureData(m(:,numb_yaw), ...
                    md(:,numb_yaw), ...
                    tau_m(:,numb_yaw), ...
                    data.time, threshold);
            else
                coupled.pitch = coupled.pitch.loadIdleMeasureData(m(:,numb_pitch), ...
                    md(:,numb_pitch), ...
                    tau_m(:,numb_pitch), ...
                    data.time, threshold, cutoff);
                coupled.roll = coupled.roll.loadIdleMeasureData(m(:,numb_roll), ...
                    md(:,numb_roll), ...
                    tau_m(:,numb_roll), ...
                    data.time, threshold, cutoff);
                coupled.yaw = coupled.yaw.loadIdleMeasureData(m(:,numb_yaw), ...
                    md(:,numb_yaw), ...
                    tau_m(:,numb_yaw), ...
                    data.time, threshold, cutoff);
            end
        end
        
        function saveToFile(coupled,name)
            %% Save on file
            if ~exist('name','var')
                name = 'data';
            end
            coupled.pitch.saveToFile(name);
            coupled.roll.saveToFile(name);
            coupled.yaw.saveToFile(name);
        end
        
        function command = getWBIlist(joint)
            %% Get string to start ControlBoardDumper
            command = ['JOINT_FRICTION = (' joint.pitch.WBIname ', ' joint.roll.WBIname, ', ' joint.yaw.WBIname ')'];
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

