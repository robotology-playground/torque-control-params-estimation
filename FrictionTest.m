
%% Load joints
joint = Motor('20141222','iCubGenova03','leg','left','hip','pitch','NJTC/WBDT/761');

%% Evalute Friction on joint with data
joint = joint.loadFriction('idle.mat',1);

%% Plot Friction

hFig = figure(2);
set(hFig, 'Position', [0 0 800 600]);
%subplot(1,2,1);
hold on
grid;
joint.friction.plotFriction();
joint.friction.plotFrictionModel();
hold off

%% Remove friction
grid;
subplot(1,2,2);
grid;
measure = joint.plotFrVsMeasure('ref.mat',1,'end');

%% Save image
currentFolder = pwd;
cd(joint.folder_path);
saveas(hFig,[joint.figureName '.fig'],'fig');
saveas(hFig,[joint.figureName '.png'],'png');
cd(currentFolder);
clear currentFolder;
%% Plot plot Coulomb - Viscous
figure(2);
joint.friction.plotParts();


%% Plot noise

hFigN = figure(3);
set(hFigN, 'Position', [0 0 800 600]);
joint.friction.plotNoise();
grid;
%% Save image
currentFolder = pwd;
cd(joint.folder_path);
saveas(hFigN,[joint.figureName '-noise.fig'],'fig');
saveas(hFigN,[joint.figureName '-noise.png'],'png');
cd(currentFolder);
clear currentFolder;