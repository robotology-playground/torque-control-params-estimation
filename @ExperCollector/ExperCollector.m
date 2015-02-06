classdef ExperCollector
    %EXPERCOLLETOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = protected)
        path;
        joint_list;
        path_motor;
    end
    
    properties
        robot;
        date;
        joint;
        data_path;
    end
    
    methods
        function obj = ExperCollector(robot, date)
            obj.robot = robot;
            if exist('date','var')
                obj.date = date;
            else
                obj.date = '';
            end
            obj.path = 'experiments';
            if strcmp(obj.date,'')
                obj.data_path = ['experiments/' obj.robot '/'];
                obj.path_motor = 'experiments';
            else
                obj.data_path = ['experiments/' obj.date '/' obj.robot '/'];
                obj.path_motor = ['experiments/' obj.date];
            end
            obj.joint_list = '';
        end
        
        function obj = setStartPath(obj,path)
            obj.path = path;
            if strcmp(obj.date,'')
                obj.data_path = [obj.path '/' obj.robot '/'];
            else
                obj.data_path = [obj.path '/' obj.date '/' obj.robot '/'];
            end
        end
        
        function obj = addMotor(obj,part, type, info1, info2)
            if exist('type','var') && exist('info1','var') && exist('info2','var')
                motor = Motor(obj.path_motor, obj.robot, part, type, info1, info2);
            elseif exist('type','var') && exist('info1','var') && ~exist('info2','var')
                motor = Motor(obj.path_motor, obj.robot, part, type, info1);
            elseif exist('type','var') && ~exist('info1','var') && ~exist('info2','var')
                motor = Motor(obj.path_motor, obj.robot, part, type);
            elseif ~exist('type','var') && ~exist('info1','var') && ~exist('info2','var')
                motor = Motor(obj.path_motor, obj.robot, part);
            end
            obj.joint = [obj.joint motor];
            
            if strcmp(obj.joint_list,'')
                obj.joint_list = motor.WBIname;
            else
                obj.joint_list = [obj.joint_list ', ' motor.WBIname];
            end
        end
        
        function command = getWBIlist(obj)
            %% Get string to start ControlBoardDumper
            command = ['JOINT_FRICTION = (' obj.joint_list ')'];
            assignin('base', 'ROBOT_DOF', size(obj.joint,2));
        end
        
        function plotAllFriction(obj)
            for i=1:size(obj.joint,2)
            % FIGURE - Friction data and estimation
            obj.joint(i).friction.savePictureToFile(obj.joint(i).path, i);
            end
        end
        
        function plotKtCoeff(obj)
            for i=1:size(obj.joint,2)
                hFig = figure(i);
                set(hFig, 'Position', [0 0 800 600]);
                obj.joint(i).plotCoeff();
                obj.joint(i).savePictureToFile(hFig,'PWMVsTorque');
            end
        end
        
        function obj = loadIdleMeasure(obj, file, threshold, cutoff)
            if ~exist('file','var')
                file = 'idle';
            end
            if ~exist('threshold','var')
                threshold = 1;
            end
            
            data = load([obj.data_path file '.mat']);
            
            for i=1:size(obj.joint,2)
                if exist('cutoff','var')
                    obj.joint(i).friction = Friction(data.q(:,i), data.qD(:,i), data.tau(:,i), data.time, threshold, cutoff);
                else
                    obj.joint(i).friction = Friction(data.q(:,i), data.qD(:,i), data.tau(:,i), data.time, threshold);
                end
            end
        end
        
        function obj = loadRefFile(obj, file)
            if ~exist('file','var')
                file = 'reference';
            end
            data = load([obj.data_path file '.mat']);
            
            for i=1:size(obj.joint,2)
                obj.joint(i) = obj.joint(i).loadReference(data);
                
            end
        end
    end
    
end

