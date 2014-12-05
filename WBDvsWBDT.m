
nFigure = 2;
type = 'T';
plot = struct;
plot.xmin = -50;
plot.xmax = 50;
plot.ymin = -9;
plot.ymax = 9;
Joints = zeros(4,1);
joints(1) = Motor('20141204','iCubGenova03','leg','left','hip','pitch',['WBD' type '+40']);
joints(1) = joints(1).loadFriction('idle.mat',1);
joints(2) = Motor('20141204','iCubGenova03','leg','left','hip','pitch',['WBD' type '+20']);
joints(2) = joints(2).loadFriction('idle.mat',1);
joints(3) = Motor('20141204','iCubGenova03','leg','left','hip','pitch',['WBD' type '-0']);
joints(3) = joints(3).loadFriction('idle.mat',1);
joints(4) = Motor('20141204','iCubGenova03','leg','left','hip','pitch',['WBD' type '-20']);
joints(4) = joints(4).loadFriction('idle.mat',1);

hFig = figure(nFigure);
set(hFig, 'Position', [0 0 800 600]);

for i=1:size(joints,2)
    subplot(1,4,i);
    axis([plot.xmin plot.xmax plot.ymin plot.ymax]);
    hold on
    grid;
    joints(i).friction.plotFriction();
    joints(i).friction.plotFrictionModel();
    hold off
    title(['At ' joints(i).name_exp '°']);
end