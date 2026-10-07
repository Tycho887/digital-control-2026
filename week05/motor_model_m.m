%% motor model parameters
close all
clear;
%% Motor parameters
L = 1e-3; % Henry
R = 1; % Ohm
Kt = 0.01; % Nm/A
Kb = Kt;   % V/(rad/s)
J = 1e-5;  % (Kg m^2) = Nm/(rad/s^2)
D = 1e-5;  % Nm/(rad/s)
gear = 9.68;
wheelRadius = 0.03; % (m)
r2m = wheelRadius/gear; % (radian/s to meter/s)
%% sim sequence
% steps 1.5V, 3V, 6V at 0.5sec interval
% format: 
% [time(sec) , Voltage left, Voltage right]
seq1 = [ 0,       0,   0; ...
         0.020, 1.5, 1.5; ...
         0.500, 3.0, 3.0; ...
         1.000, 6.0, 6.0; ...
         1.500, 0.0, 0.0; ...
         2.000, 0,   0; ...
       ];
% Three 10ms pulses of 6V with 300ms interval
% format: 
% [time(sec) , Voltage left, Voltage right]
seq2 = [0, 0, 0; ...
         0.05, 6, 6; ...
         0.06, 0, 0; ...
         0.3, 6, 6; ...
         0.31, 0, 0; ...
         0.6, 6, 6; ...
         0.61, 0, 0; ...
       ];
seq = seq1;
%% simulate (Steps)
model ='motor_model_23b';
seq = seq1;         % switch to 2-step sequence
aa = sim(model, 2); % output will be called 'aa', simulate in 2 seconds
%% simulink version of the loaded model (just debug)
info = Simulink.MDLInfo(model);
disp(info.ReleaseName); % Displays release version (e.g., 'R2023b')
%% plot result (Steps)
h = figure(10);
hold off
plot(aa.tout, aa.so.Data(:,1),'LineWidth',1.5)
hold on
plot(aa.tout, aa.so.Data(:,2),'LineWidth',1)
plot(aa.tout, aa.so.Data(:,4)/100,'LineWidth',1.5)
legend('Motor voltage (V)', 'Motor current (A)', 'Velocity (rad/s / 100)','Location','northwest')
grid on
axis([0,1.6,-0.1, 6.3])
xlabel('Time (sec)')
title('Steady state and slow transition')
grid on
saveas(h,'motor_model_steps_01.png')

%% Simulate (spikes)
seq = seq2;             % switch to spike sequence
bb = sim(model, 0.75);  % output will be in 'bb', simulate in 0.75 seconds
%% plot result (spikes)
h = figure(11);
hold off
plot(bb.tout, bb.so.Data(:,1),'LineWidth',2)
hold on
plot(bb.tout, bb.so.Data(:,2),'-x','LineWidth',2)
plot(bb.tout, bb.so.Data(:,3),'LineWidth',2)
legend('Motor voltage (V)', 'Motor current (A)', 'Velocity (m/s)','Location','best')
grid on
axis([0.048,0.063,-0.75, 7])
xlabel('Time (sec)')
title('Fast transition (first spike)')
grid on
saveas(h,'motor_model_spikes_01.png')
%
%
