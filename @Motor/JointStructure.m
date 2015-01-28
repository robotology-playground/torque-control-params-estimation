function joint = JointStructure(joint)
%JOINTSTRUCTURE Summary of this function goes here
%   Detailed explanation goes here

% For LEG
% 1	hip_pitch	Hip     pitch	When the leg principal axis is aligned with gravity (front-back movement)
% 2	hip_roll	Hip     roll	Lateral movement (when leg aligned with gravity)
% 3	hip_yaw     Hip     yaw     Rotation along the leg/tight principal axis
% 4	knee        Knee	--
% 5	ankle_pitch	Ankle	pitch	When the calf is aligned with gravity
% 6	ankle_roll  Ankle	roll	When the calf is aligned with gravity


%joint = struct;
% HEAD

%Convert part in number
part_number = 0;
if strcmp(joint.part,'head')
    part_number = 1;
elseif strcmp(joint.part,'torso')
    part_number = 2;
elseif strcmp(joint.part,'arm')
    part_number = 3;
elseif strcmp(joint.part,'leg')
    part_number = 4;
end

%Convert type in number
if strcmp(joint.type,'left')
    type_number = 1;
elseif strcmp(joint.type,'right')
    type_number = 2;
elseif(joint.info1 ~= 0)
    type_number = joint.pitchRollYawNumber(joint.type);
end

%Convert info1 in number
info1_number = 0;
if strcmp(joint.info1,'hip')
    info1_number = 1;
elseif strcmp(joint.info1,'knee')
    info1_number = 4;
elseif strcmp(joint.info1,'ankle')
    info1_number = 3;
elseif(joint.info1 ~= 0)
    info1_number = joint.pitchRollYawNumber(joint.info1);
elseif strcmp(joint.info1,'elbow')
    info1_number = 4;
end

%Convert info2 in number
info2_number = -1;
if ~strcmp(joint.info2,'')
    info2_number = joint.pitchRollYawNumber(joint.info2);
end

DOF_START_TORSO = 0;
DOF_START_ARM_LEFT = joint.N_TORSO;
DOF_START_ARM_RIGHT = DOF_START_ARM_LEFT + joint.N_HAND;
DOF_START_LEG_LEFT = DOF_START_ARM_RIGHT + joint.N_HAND;
DOF_START_LEG_RIGHT = DOF_START_LEG_LEFT + joint.N_LEG;
joint.number = 0;       % For ROBOT_DOF = 25
joint.path = [joint.start_folder '/' joint.robot];
joint.number_part = 0;
switch(part_number)
    case 2      % Torso
        joint.group_select = joint.part;
        joint.path = [joint.path '/' joint.part '/' joint.type];
        joint.number = DOF_START_TORSO + type_number;
        joint.number_part = type_number;
    case 3      % Arm
        joint.group_select = [joint.type '_' joint.part];
        joint.path = [joint.path '/' joint.part '/' joint.type '/' joint.info1];
        joint.number = info1_number;
        joint.number_part = info1_number;
        switch(type_number)
            case 1
                joint.number = joint.number + DOF_START_LEG_LEFT;
            case 2
                joint.number = joint.number + DOF_START_LEG_RIGHT;
        end
    case 4      % Leg
        joint.group_select = [joint.type '_' joint.part];
        switch(info1_number)
            case 1     % Case Hip
                joint.path = [joint.path '/' joint.part '/' joint.type '/' joint.info1 '/' joint.info2];
                joint.number = info2_number;
            case 3     % Case ankle
                joint.path = [joint.path '/' joint.part '/' joint.type '/' joint.info1 '/' joint.info2];
                joint.number = ANKLE_START + info2_number;
            case 4     % Case knee
                joint.path = [joint.path '/' joint.part '/' joint.type '/' joint.info1];
                joint.number = info1_number;
        end
        joint.number_part = joint.number;
        switch(type_number)
            case 1
                joint.number = joint.number + DOF_START_LEG_LEFT;
            case 2
                joint.number = joint.number + DOF_START_LEG_RIGHT;
        end
end
joint.path = [joint.path '/'];
% Select column
I_matrix = eye(joint.robot_dof);
if joint.robot_dof > 1
    joint.select = I_matrix(:,joint.number);
else
    joint.select = 1;
end
