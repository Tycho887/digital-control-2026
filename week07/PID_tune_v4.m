%% PID_tune_v4.m
% Self-contained: physical constants (formerly from week07_pid_tune.m)
% + voltage-limited wc-sweep tuning (PID3.m), with the Simulink push.
% Requires week07_wheel_model.slx on the path.

clear; clc;

%% ===== 1. Physical constants =====
L = 0.00768482381376416;  R = 6.41139804096171;
Kt = 0.010293561274142;   Kb = Kt;
J = 5.46186035789905e-06; D = 6.52401278374518e-07;
%L = 0.0033; R = 2.86;
%Kb = 0.00798; Kt = Kb;
%J = 5.46e-6; D = 3.38e-6;


gear = 9.68;  wheelRadius = 0.03;
Ts = 0.001;   voltageLimit = 9;
referenceSpeed = 0.1;   stepTime = 0.2;

%% ===== 2. Discrete plant + 1-sample delay =====
plant = tf(Kt, [L*J, L*D + R*J, R*D + Kt*Kb]);
plant = c2d(plant, Ts, 'zoh');
z = tf('z', Ts);
plant_delayed = plant * z^(-1);

%% ===== 3. Load Simulink model workspace =====
model = 'week07_wheel_model';
load_system(fullfile(fileparts(mfilename('fullpath')), [model '.slx']));
workspace = get_param(model, 'ModelWorkspace');

%% ===== 4. Tuning: wc-sweep on the delayed plant (PID3 logic) =====
e_max = referenceSpeed * (gear / wheelRadius);

C_template = pid(1, 1, 1, 0.002, Ts, ...
    'IFormula','ForwardEuler', ...
    'DFormula','BackwardEuler');

opts = pidtuneOptions('PhaseMargin', 50, 'DesignFocus','balanced');

target_peak_V = 0.99 * voltageLimit;
wc_candidates = linspace(25, 150, 126);
wc_opt = wc_candidates(1);

for wc_test = wc_candidates
    C_test  = pidtune(plant_delayed, C_template, wc_test, opts);
    b0_test = tfdata(tf(C_test), 'v');
    if b0_test(1) * e_max <= target_peak_V
        wc_opt = wc_test;
    else
        break;
    end
end

[C, info] = pidtune(plant_delayed, C_template, wc_opt, opts);
fprintf('Selected wc = %.1f rad/s | Expected peak voltage = %.2f V\n', ...
    wc_opt, b0_test(1) * e_max);
disp(C); disp(info);

%% ===== 5. Extract CCDE coefficients =====
[num, den] = tfdata(tf(C), 'v');
num = [num, zeros(1, 3 - length(num))];
den = [den, zeros(1, 3 - length(den))];

b0 = num(1); b1 = num(2); b2 = num(3);
a1 = den(2); a2 = den(3);

fprintf('u[k] = %.6f*e[k] + %.6f*e[k-1] + %.6f*e[k-2] - (%.6f*u[k-1] + %.6f*u[k-2])\n', ...
    b0, b1, b2, a1, a2);

%% ===== 6. Push into Simulink =====
wheel = struct('L', L, 'R', R, 'Kt', Kt, 'Kb', Kb, 'J', J, 'D', D, ...
    'gear', gear, 'wheelRadius', wheelRadius, 'Kp', C.Kp, 'Ts', Ts, ...
    'voltageLimit', voltageLimit);
assignin(workspace, 'wheel', wheel);
assignin(workspace, 'referenceSpeed', referenceSpeed);
assignin(workspace, 'stepTime', stepTime);

%% ===== 7. Linear closed-loop step response (with delay) =====
t_sim = 0:Ts:0.6;
sys_cl = feedback(C * plant, z^(-1));
opt_step = stepDataOptions('StepAmplitude', referenceSpeed);
[y_lin, t_lin] = step(sys_cl, t_sim, opt_step);

%% ===== 8. CCDE simulation loop =====
[numP, denP] = tfdata(plant, 'v');
q1 = numP(2); q2 = numP(3);
p1 = denP(2); p2 = denP(3);

N_steps = length(t_sim);
r_vel = zeros(1, N_steps);
r_vel(t_sim >= stepTime) = referenceSpeed;
r_mot = r_vel * (gear / wheelRadius);

e     = zeros(1, N_steps);
u     = zeros(1, N_steps);
u_act = zeros(1, N_steps);
y_mot = zeros(1, N_steps);

for k = 3:N_steps
    y_mot(k) = -p1*y_mot(k-1) - p2*y_mot(k-2) + q1*u_act(k-1) + q2*u_act(k-2);
    e(k)     = r_mot(k) - y_mot(k-1);
    u(k)     = b0*e(k) + b1*e(k-1) + b2*e(k-2) - (a1*u(k-1) + a2*u(k-2));
    u_act(k) = min(max(u(k), -voltageLimit), voltageLimit);
end

y_wheel = y_mot * (wheelRadius / gear);

%% ===== 9. Plot =====
figure('Name','PID_tune_v4: Voltage-Limited Tuning');
subplot(2,1,1);
stairs(t_sim, r_vel, 'k--', 'LineWidth',1.2); hold on;
stairs(t_lin + stepTime, y_lin, 'b-', 'LineWidth',1.5);
stairs(t_sim, y_wheel, 'r:', 'LineWidth',1.8);
xlim([0 0.6]); grid on;
ylabel('Wheel Velocity (m/s)');
title(sprintf('Step Response (wc = %.1f rad/s)', wc_opt));
legend('Reference','Linear Closed-Loop','CCDE Loop','Location','best');

subplot(2,1,2);
stairs(t_sim, u_act, 'm-', 'LineWidth',1.5);
yline( voltageLimit,'k--'); yline(-voltageLimit,'k--');
xlim([0 0.6]); ylim([-voltageLimit-1, voltageLimit+1]); grid on;
xlabel('Time (s)'); ylabel('Motor Voltage (V)');
title('Actuator Control Effort');

b0_opt = tfdata(tf(C), 'v'); b0_opt = b0_opt(1);
fprintf('Selected wc = %.1f rad/s | Peak voltage = %.2f V\n', ...
    wc_opt, b0_opt * e_max);