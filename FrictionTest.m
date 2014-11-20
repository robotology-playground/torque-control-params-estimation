
%% Load value in idle for friction estimatation
load([joint.folder_path 'idle.mat']);
Fr = Friction(out(:,1),out(:,2),out(:,3),time,1);
TH = 1;
%% Plot Friction

subplot(1,2,1);
hold on
Fr.plotFriction();
grid;
coeff = Fr.getFrictionData(TH);
hold off

%% Remove friction
load([joint.folder_path 'ref.mat']);
time_set = Time;
%figure;
qdot = out(1:time_set*100,2);
torque = out(1:time_set*100,3);
pwm = out(1:time_set*100,4);
pwm_jtc = out(1:time_set*100,5);
friction = zeros(size(qdot,1),1);
friction(qdot < -TH) = coeff.AN(2) + coeff.AN(1)*qdot(qdot < -TH);
friction(qdot > TH) = coeff.AP(2) + coeff.AP(1)*qdot(qdot > TH);

%Plot data
subplot(1,2,2);
plot(pwm,torque-friction,'.');
xlabel('PWM','Interpreter','tex');
ylabel('\tau','Interpreter','tex');
grid;
hold on
a = lineRegress(pwm,torque-friction);
plot(pwm,pwm*a(1),'r-');
%plot(pwm_jtc,torque-friction,'g.');
%a_jtc = lineRegress(pwm_jtc,torque-friction);
%plot(pwm_jtc,pwm_jtc*a_jtc(1),'m-');
hold off

%% Plot plot Coulomb - Viscous
figure;
Fr.plotTree(coeff);


%% 

figure;
plot(pwm,torque,'.');
hold on;
plot(pwm,friction,'r.');
plot(pwm,torque-friction,'m.');
grid;
hold off;