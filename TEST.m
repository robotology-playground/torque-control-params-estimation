

robot = Robot('iCubGenova03', 'AAA', '/Users/Raffaello/iit/codyco-superbuild');

robot.joints = robot.getJoint('l_ankle_pitch');
robot.joints = [robot.joints robot.getCoupledJoints('torso')];
%robot.joints = [robot.joints robot.getCoupledJoints('l_shoulder')];
robot.configure('JOINT_FRICTION','false');
robot.buildFolders();

joint = robot.joints{1};
joint.motor