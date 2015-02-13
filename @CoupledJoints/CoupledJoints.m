classdef CoupledJoints < ExperimentCollector
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
        
        function savePictureKt(coupled, counter)
            axes_data = [];
            for i=1:size(coupled.joint,2)
                coupled.joint(i).savePictureKt(counter);
                axes_data = [axes_data gca];
                counter = counter + 1;
            end
            
            hCollect = figure(counter); %create new figure
            set(hCollect, 'Position', [0 0 800 600]);
            for i=1:size(coupled.joint,2)
                subPlotData = subplot(1,size(coupled.joint,2),i); %create and get handle to the subplot axes
                figData = get(axes_data(i),'children');
                title(coupled.joint(i).WBIname);
                grid;
                copyobj(figData,subPlotData);
            end
            currentFolder = pwd;
            cd(coupled.path);
            if ~exist('figureName','var')
                figureName = 'PWMVsTorque';
            end
            if ~strcmp(coupled.name_experiment,'')
                figureName = [figureName '-' coupled.name_experiment];
            end
            saveas(hCollect,[figureName '.fig'],'fig');
            saveas(hCollect,[figureName '.png'],'png');
            cd(currentFolder);
        end
        
        function savePictureFriction(coupled, counter)
            axes_data = [];
            for i=1:size(coupled.joint,2)
                coupled.joint(i).savePictureFriction(counter);
                axes_data = [axes_data gca];
                counter = counter + 1;
            end
            
            hCollect = figure(counter); %create new figure
            set(hCollect, 'Position', [0 0 800 600]);
            for i=1:size(coupled.joint,2)
                subPlotData = subplot(1,size(coupled.joint,2),i); %create and get handle to the subplot axes
                figData = get(axes_data(i),'children');
                title(coupled.joint(i).WBIname);
                grid;
                copyobj(figData,subPlotData);
            end
            currentFolder = pwd;
            cd(coupled.path);
            if ~exist('figureName','var')
                figureName = 'friction';
            end
            if ~strcmp(coupled.name_experiment,'')
                figureName = [figureName '-' coupled.name_experiment];
            end
            saveas(hCollect,[figureName '.fig'],'fig');
            saveas(hCollect,[figureName '.png'],'png');
            cd(currentFolder);
        end
        
        function coupled = loadIdleMeasure(coupled, file, cutoff)
            if ~exist('file','var')
                file = 'idle';
            end
            coupled.name_experiment = file;
            data = load([coupled.path file '.mat']);
            data.m     = (coupled.T^-1*data.q')';
            data.mD    = (coupled.T^-1*data.qD')';
            data.mDD    = (coupled.T^-1*data.qDD')';
            data.tauM = (coupled.T'*data.tau')';
            
            for i=1:size(coupled.joint,2)
                if exist('cutoff','var')
                    coupled.joint(i).friction = Friction(data.m(:,i), data.mD(:,i), data.mDD(:,i), data.tauM(:,i), data.time, cutoff);
                    coupled.joint(i).friction = coupled.joint(i).friction.setExperiment(coupled.name_experiment);
                else
                    coupled.joint(i).friction = Friction(data.m(:,i), data.mD(:,i), data.mDD(:,i), data.tauM(:,i), data.time);
                    coupled.joint(i).friction = coupled.joint(i).friction.setExperiment(coupled.name_experiment);
                end
            end
        end
        
        function coupled = loadRefMeasure(coupled, file)
            if ~exist('file','var')
                file = 'ref';
            end
            data = load([coupled.path file '.mat']);
            
            data.q     = (coupled.T^-1*data.q')';
            data.qD    = (coupled.T^-1*data.qD')';
            data.qDD    = (coupled.T^-1*data.qDD')';
            data.tau = (coupled.T'*data.tau')';
            
            for i=1:size(coupled.joint,2)
                temp = struct;
                temp.q = data.q(:,i);
                temp.qD = data.qD(:,i);
                temp.qDD = data.qDD(:,i);
                temp.tau = data.tau(:,i);
                temp.PWM = data.PWM;
                temp.Current = data.Current;
                temp.time = data.time;
                coupled.joint(i) = coupled.joint(i).loadReference(temp);
            end
        end
    end
    
end

