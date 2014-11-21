function joint = JointStructure( joint, robot, part, type, info1, info2 )
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
joint.number = 3;
joint.path = [type '_' part];
joint.folder_path = [robot '/' part '/' type '/' info1];
joint.robotName = robot;
number = 1;

if strcmp(part,'hand')
    if strcmp(type,'right')
        joint.number = joint.number + 5;
    end
elseif strcmp(part,'leg')
    joint.number = joint.number + 10;
    if strcmp(type,'right')
        joint.number = joint.number + 6;
    end
  
    if strcmp(info1,'hip')
        if strcmp(info2,'roll')
            number = number + 1;
        elseif strcmp(info2,'yaw')
            number = number + 2;
        end
        joint.folder_path = [joint.folder_path '/' info2];
    elseif strcmp(info1,'knee')
        number = number + 3;
    elseif strcmp(info1,'ankle')
        number = number + 4;
        if strcmp(info2,'roll')
            number = number + 1;
        end
        joint.folder_path = [joint.folder_path '/' info2];
    end
end

joint.number = joint.number + number;
joint.folder_path = [joint.folder_path '/'];

end

