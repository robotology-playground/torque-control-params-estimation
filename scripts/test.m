
% AP = lineRegress(qD(qD > th_vel/2), tau(qD > th_vel/2));
% AN = lineRegression(qD(qD < -th_vel/2), tau(qD < -th_vel2));

% fr = zeros(size(qD,1),1);
% fr(qdot < -th_vel/2) = obj.KcN + obj.KvN*qD(qD < -th_vel/2);
% fr(qdot > th_vel/2) = obj.KcP + obj.KvP*qD(qD > th_vel/2);


%%
th_vel = 0;

%fr = zeros(size(qD,1),1);
qD_d = qD((qD > th_vel/2) & (qDD > 0));
qD_d = [qD_d; qD((qD < -th_vel/2) & (qDD < 0))];

qDD_d = qDD((qD > th_vel/2) & (qDD > 0));
qDD_d = [qDD_d; qDD((qD < -th_vel/2) & (qDD < 0))];

fr = tau((qD > th_vel/2) & (qDD > 0));
fr = [fr; tau((qD < -th_vel/2) & (qDD < 0))];

%fr((qD > th_vel/2) & (qDD > 0)) = tau((qD > th_vel/2) & (qDD > 0));
%fr((qD < -th_vel/2) & (qDD < 0)) = tau((qD < -th_vel/2) & (qDD < 0));
%%
%plot3(qD,qDD,tau,'.');
plot(qD,tau,'.');
%plot(qDD,tau,'.');
hold on;
grid;
plot(qD_d, fr, 'r.');
%plot3(qD_d, qDD_d, fr, 'r.');
hold off;