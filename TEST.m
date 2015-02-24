

robot = Robot('iCubGenova03', 'AAA', '/Users/Raffaello/iit/codyco-superbuild');

robot.joints = robot.getJoint('l_ankle_pitch');
%robot.joints = [robot.joints robot.getCoupledJoints('torso')];
%joint_list = [joint_list robot.getCoupledJoints('l_shoulder')];
robot.buildFolders();
robot.configure('JOINT_FRICTION');