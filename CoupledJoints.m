
robotName = 'iCubGenova03';
part = 'arm';
type = 'left';
date = '20150123';
joint = Motor(date,robotName,part, type,'pitch');
joint = [joint Motor(date,robotName, part, type,'roll')];
joint = [joint Motor(date,robotName, part, type,'yaw')];
folder_path = [date '/' robotName '/' part '/' type '/'];
joint_path = type;
joint_number = [joint(1).number joint(2).number joint(3).number];
Time = 60;
%% 
name = 'idle';

%%% Reference configuration
Reference = struct;

if strcmp(name,'idle')
    Reference.sinAmp = 0;
    Reference.sinFreq = 0;
    Reference.sinBias = 0;
elseif strcmp(name,'ref')
    Reference.sinAmp = 5;
    Reference.sinFreq = 0.05;
    Reference.sinBias = 0;
end
name_file = [name ''];
%% Set Variable
setenv('YARP_DATA_DIRS', '/Users/Raffaello/iit/codyco-superbuild/build/install/share/codyco');
localName = 'simulink_raffa';
setenv('YARP_ROBOT_NAME', robotName);
ROBOT_DOF = 25;
robotName = 'icub';
Ts = 0.01;

%% Save on file
if exist('tout','var')
    time = tout;
    clear tout;
end
save([folder_path name_file '.mat'],'logsout','time');
disp(['SAVED ON ' folder_path name_file '.mat']);

%% Coupled Friction
% TORSO
% R = 0.04;
% r = 0.022;
% T = [r/R r/(2*R) r/(2*R); 
%       0    1/2     1/2;
%       0   -1/2     1/2];

t = 0.625;
T = [-1 0 0;
    -1 -t 0;
    0 t -t];
  

m     = (T^-1*logsout.get('q').Values.Data')';
md    = (T^-1*logsout.get('qD').Values.Data')';
tau_m = (T'*logsout.get('tau').Values.Data')';
time  = logsout.get('qD').Values.Time;
friction = Friction(m(:,1) ,md(:,1),tau_m(:,1), time, 0.1);
friction = [friction Friction(m(:,2) ,md(:,2),tau_m(:,2), time, 0.1)];
friction = [friction Friction(m(:,3) ,md(:,3),tau_m(:,3), time, 0.1)];

%%
% 
% Friction(logsout.get('q').Values.Data(:,1),...
%         logsout.get('qD').Values.Data(:,1),...
%         logsout.get('tau').Values.Data(:,1),...
%         logsout.get('qD').Values.Time, 1).plotFriction();


%% Plot Friction

hFig = figure(1);
set(hFig, 'Position', [0 0 800 600]);

subplot(1,3,1);
hold on
grid;
friction(3).plotFriction();
friction(3).plotFrictionModel();
title('Pitch');
hold off

subplot(1,3,2);
hold on
grid;
friction(2).plotFriction();
friction(2).plotFrictionModel();
title('Roll');
hold off

subplot(1,3,3);
hold on
grid;
friction(1).plotFriction();
friction(1).plotFrictionModel();
title('Yaw');
hold off