
joint_data = struct;

joint_data.q_j = logsout.get('q').Values.Data(:,joint.number);
joint_data.qd_j = logsout.get('qD').Values.Data(:,joint.number);
joint_data.qdd_j = logsout.get('qDD').Values.Data(:,joint.number);