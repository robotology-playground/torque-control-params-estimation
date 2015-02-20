classdef (Abstract) Joint
    %JOINT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods (Abstract)
        saveParameters(obj);
        obj = loadParameters(obj, file);
        obj = loadIdleMeasure(obj, file, cutoff);
        obj = loadRefMeasure(obj, file);
        obj = setRatio(obj, Voltage, range_pwm);
        list = getJointList(obj);
    end
    
end

