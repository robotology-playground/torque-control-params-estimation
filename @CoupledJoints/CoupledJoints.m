classdef CoupledJoints < ExperimentCollector & Joint
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        T;
        name_experiment = '';
        group_select;
    end
    
    methods
        function coupled = CoupledJoints(start_folder, robotName, part, type)
            coupled = coupled@ExperimentCollector(robotName);
            coupled = coupled.setStartPath(start_folder);
            if ~exist('type','var')
                coupled = coupled.addMotor(part, 'yaw');
                coupled = coupled.addMotor(part, 'roll');
                coupled = coupled.addMotor(part, 'pitch');
                coupled.path = [coupled.path part '/'];
            else
                coupled = coupled.addMotor(part, type, 'pitch');
                coupled = coupled.addMotor(part, type, 'roll');
                coupled = coupled.addMotor(part, type, 'yaw');
                coupled.path = [coupled.path part '/' type '/'];
            end
            % Save group select
            coupled.group_select = coupled.joint(1).group_select;
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
        end
        
        function coupled = changeTmatrix(coupled, T)
            coupled.T = T;
        end
        
        function coupled = setRatio(coupled, Voltage, range_pwm)
            for i=1:size(coupled.joint,2)
                coupled.joint(i) = coupled.joint(i).setRatio(Voltage, range_pwm);
            end
        end
        
        function coupled = setFrictionToCenter(coupled)
            for i=1:size(coupled.joint,2)
                coupled.joint(i) = coupled.joint(i).setFrictionToCenter();
            end
        end
        
        function plotKt(coupled, option)
            if ~exist('option','var')
                option = '.';
            end
            for i=1:size(coupled.joint,2)
                coupled.joint(i).plotKt(option);
            end
        end
        
        function hCollect = plotCollect(coupled, counter, axes_data, figureName)
            hCollect = figure(counter); %create new figure
            set(hCollect, 'Position', [0 0 800 600]);
            for i=1:size(coupled.joint,2)
                subPlotData = subplot(1,size(coupled.joint,2),i); %create and get handle to the subplot axes
                figData = get(axes_data(i),'children');
                title(coupled.joint(i).getJointList());
                grid;
                copyobj(figData,subPlotData);
            end
            currentFolder = pwd;
            cd(coupled.path);
            if ~strcmp(coupled.name_experiment,'')
                figureName = [figureName '-' coupled.name_experiment];
            end
            saveas(hCollect,[figureName '.fig'],'fig');
            saveas(hCollect,[figureName '.png'],'png');
            cd(currentFolder);
        end
    end
    
end

