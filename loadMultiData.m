

joint = Motor('experiments','iCubGenova01','leg','left','hip','roll');

data = ls([ joint.path 'idle-*.mat']);
data = strsplit(data,'\n');
KcP = zeros(size(data,2)-1,1);
KcN = zeros(size(data,2)-1,1);
KvP = zeros(size(data,2)-1,1);
KvN = zeros(size(data,2)-1,1);
Offset = zeros(size(data,2)-1,1);
q = [];
qD = [];
qDD = [];
tau = [];
time = [];
figure;
hold on
cc=hsv(size(data,2)-1);
for i=1:size(data,2)-1
    name = strsplit(data{i},{'/','.'});
    name = name{end-1};
    m = load([joint.path name '.mat']);
    q = [q; m.q];
    qD = [qD; m.qD];
    qDD = [qDD; m.qDD];
    tau = [tau; m.tau-mean(m.tau)];
    %tau = [tau; m.tau];
    if i == 1
        time = m.time;
    else
        time = [time; time(end)+m.time];
    end
    %plot(m.qD,m.tau-mean(m.tau),'.','color',cc(i,:));
    plot(m.qD,m.tau,'.','color',cc(i,:));
%     joint = joint.loadIdleMeasure(name);
%     joint = joint.setFrictionToCenter();
%     joint.friction.plotFriction();
%     KcP(i) = joint.friction.KcP;
%     KcN(i) = joint.friction.KcN;
%     KvP(i) = joint.friction.KvP;
%     KvN(i) = joint.friction.KvN;
%     Offset(i) = joint.friction.offset;
end
joint.friction = Friction(q,qD,qDD,tau,time);
grid;
hold off

figure;
hold on
joint.friction.plotFriction();
joint.friction.plotFrictionModel();
grid;
hold off
%[~, i_max] = max(KcP);

