classdef (Abstract) Joint
    %JOINT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods (Abstract)
        %loadParameters(obj);
        %obj = loadIdleMeasure(obj, file, cutoff);
        %obj = loadRefMeasure(obj, file);
        list = getJointList(obj);
    end
    
end

