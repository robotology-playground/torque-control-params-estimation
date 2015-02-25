

robot = Robot('iCubGenova03', 'Test2', '/Users/Raffaello/iit/codyco-superbuild');

robot.joints = robot.getJoint('l_ankle_roll');
%robot.joints = [];
%robot.joints = [robot.joints robot.getCoupledJoints('torso')];
%robot.joints = [robot.joints robot.getCoupledJoints('l_shoulder')];
robot.configure('JOINT_FRICTION','false');
robot.buildFolders();

%robot = robot.loadData('idle');
%robot = robot.loadData('ref');

%joint = robot.joints{1};
%joint.motor