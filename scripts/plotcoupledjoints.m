

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