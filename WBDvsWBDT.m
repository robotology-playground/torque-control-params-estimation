
nFigure = 1;
type = 'WBD';
plot = struct;
plot.xmin = -50;
plot.xmax = 50;
plot.ymin = -9;
plot.ymax = 9;
Joints = zeros(4,1);
dataExp = '20141211';
robotName = 'iCubGenova01';

joints(1) = Motor(dataExp,robotName,'leg','left','hip','pitch',[type '/+60d']);
joints(1) = joints(1).loadFriction('idle.mat',1);
joints(2) = Motor(dataExp,robotName,'leg','left','hip','pitch',[type '/+40d']);
joints(2) = joints(2).loadFriction('idle.mat',1);
joints(3) = Motor(dataExp,robotName,'leg','left','hip','pitch',[type '/+20d']);
joints(3) = joints(3).loadFriction('idle.mat',1);
joints(4) = Motor(dataExp,robotName,'leg','left','hip','pitch',[type '/0d']);
joints(4) = joints(4).loadFriction('idle.mat',1);
joints(5) = Motor(dataExp,robotName,'leg','left','hip','pitch',[type '/-20d']);
joints(5) = joints(5).loadFriction('idle.mat',1);

hFig = figure(nFigure);
set(hFig, 'Position', [0 0 800 600]);

for i=1:size(joints,2)
    subplot(1,size(joints,2),i);
    axis([plot.xmin plot.xmax plot.ymin plot.ymax]);
    hold on
    grid;
    joints(i).friction.plotFriction();
    joints(i).friction.plotFrictionModel();
    hold off
    title(['At ' joints(i).name_exp '°']);
end

%% Save image
currentFolder = pwd;
cd([joints(1).folder_path '../']);
saveas(hFig,[robotName '-' type '.fig'],'fig');
saveas(hFig,[robotName '-' type '.png'],'png');
cd(currentFolder);
clear currentFolder;