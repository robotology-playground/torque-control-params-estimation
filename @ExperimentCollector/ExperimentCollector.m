classdef ExperimentCollector
    %EXPERCOLLETOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = protected)
        start_path;
        joint_list;
    end
    
    properties
        robot;
        joint;
        path;
    end
    
    methods
        function obj = ExperimentCollector(robot)
            obj.robot = robot;
            obj.start_path = 'experiments';
            obj.path = ['experiments/' obj.robot '/'];
            obj.joint_list = '';
        end
        
        function obj = setStartPath(obj,start_path)
            obj.start_path = start_path;
            obj.path = [obj.start_path '/' obj.robot '/'];
        end
        
        function obj = addMotor(obj,part, type, info1, info2)
            if exist('type','var') && exist('info1','var') && exist('info2','var')
                motor = Motor(obj.start_path, obj.robot, part, type, info1, info2);
            elseif exist('type','var') && exist('info1','var') && ~exist('info2','var')
                motor = Motor(obj.start_path, obj.robot, part, type, info1);
            elseif exist('type','var') && ~exist('info1','var') && ~exist('info2','var')
                motor = Motor(obj.start_path, obj.robot, part, type);
            elseif ~exist('type','var') && ~exist('info1','var') && ~exist('info2','var')
                motor = Motor(obj.start_path, obj.robot, part);
            end
            obj.joint = [obj.joint motor];
            
            if strcmp(obj.joint_list,'')
                obj.joint_list = motor.WBIname;
            else
                obj.joint_list = [obj.joint_list ', ' motor.WBIname];
            end
        end
        
        function list = getJointList(obj)
            list = obj.joint_list;
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
        
        function obj = loadIdleMeasure(obj, file, cutoff)
            if ~exist('file','var')
                file = 'idle';
            end
            
            data = load([obj.path file '.mat']);
            
            for i=1:size(obj.joint,2)
                if exist('cutoff','var')
                    obj.joint(i).friction = Friction(data.q(:,i), data.qD(:,i), data.qDD(:,i), data.tau(:,i), data.time, cutoff);
                    obj.joint(i).friction = obj.joint(i).friction.setExperiment(file);
                else
                    obj.joint(i).friction = Friction(data.q(:,i), data.qD(:,i), data.qDD(:,i), data.tau(:,i), data.time);
                    obj.joint(i).friction = obj.joint(i).friction.setExperiment(file);
                end
            end
        end
        
        function obj = loadRefMeasure(obj, file)
            if ~exist('file','var')
                file = 'ref';
            end
            data = load([obj.path file '.mat']);
            
            for i=1:size(obj.joint,2)
                obj.joint(i) = obj.joint(i).loadReference(data);
                
            end
        end
    end
    
end

