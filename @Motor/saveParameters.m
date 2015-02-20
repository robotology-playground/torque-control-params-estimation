function saveParameters(joint, file)
    if ~exist('file','var')
        file = 'parameters';
    end
    m = matfile([joint.path file '.mat'],'Writable',true);
    m.Voltage = joint.Voltage;
    m.range_pwm = joint.range_pwm;
end