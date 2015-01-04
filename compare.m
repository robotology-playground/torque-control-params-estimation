
plot = struct;
plot.xmin = -50;
plot.xmax = 50;
plot.ymin = -9;
plot.ymax = 9;
dataExp = '20141211';
robotName = 'iCubGenova01';

%% Compare

jointWBD = Motor(dataExp,robotName,'leg','left','knee','','WBDT/pp');
jointWBD = jointWBD.loadFriction('idle.mat',1);
jointWBDT = Motor(dataExp,robotName,'leg','left','knee','','WBDT/pp2');
jointWBDT = jointWBDT.loadFriction('idle.mat',1);

hFig = figure(1);
set(hFig, 'Position', [0 0 800 600]);

subplot(1,2,1);
hold on
grid;
jointWBD.friction.plotFriction();
jointWBD.friction.plotFrictionModel();
hold off

subplot(1,2,2);
hold on
grid;
jointWBDT.friction.plotFriction();
jointWBDT.friction.plotFrictionModel();
hold off

%% Save image
currentFolder = pwd;
cd([jointWBD.folder_path '']);
saveas(hFigN,[joint.figureName '.fig'],'fig');
saveas(hFigN,[joint.figureName '.png'],'png');
cd(currentFolder);
clear currentFolder;