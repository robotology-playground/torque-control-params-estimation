function joint = loadParameters(joint, file)
    if ~exist('file','var')
        file = 'parameters';
    end
    m = load([joint.path file '.mat']);
    joint.Voltage = m.Voltage;
    joint.range_pwm = m.range_pwm;
    joint = joint.setRatio(joint.Voltage, joint.range_pwm);
end