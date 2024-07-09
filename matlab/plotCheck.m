fanDyn=load('FanDynData.txt');
fanDyn=fanDyn';
stepResponse=rescale(fanDyn(:,2));
time=fanDyn(:,1);
signal=-fanDyn(:,3);
signal=rescale(signal);
signalAug=(signal-mean(signal));% +0.01625
signalAug=rescale(signalAug);
figure
plot(time,signalAug);
hold on 
yyaxis right
plot(time,stepResponse);

N = length(fanDyn); % defining number of elements
Y = signal;
u = stepResponse;

%% System ID
% Estimating delay term

Ts = 0.015;
data = iddata(signalAug, stepResponse, Ts);
delay_samples = delayest(data);
delay_time = delay_samples * Ts;
% [signalAug stepResponse]

sysTF = tfest(data, 1, 0, delay_time);
figure
compare(data, sysTF);
figure
resid(sysTF,data);

%% Validation
% shows that the sysf is accurate enough and we don't need to model
% % noise
% sysInit = idproc('P1D','TimeUnit', 'seconds');
% 
% sysInit.Structure.Kp.Value = 1;
% sysInit.Structure.Kp.Minimum = 0;
% sysInit.Structure.Tp1.Value = 30;
% sysInit.Structure.Tp1.Maximum = 100;
% sysInit.Structure.Td.Value = 0.2;
% sysInit.Structure.Td.Minimum = 0;
% sysInit.Structure.Td.Maximum = 20;
% 
% opt = procestOptions('DisturbanceModel', 'Arma1');
% sysP1D = procest(data, sysInit, opt);
% figure
% resid(sysP1D,data)
% figure
% step(sysTF,10); grid on
[num,den] = tfdata(sysTF)


