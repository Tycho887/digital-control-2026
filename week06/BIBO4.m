% DC Motor Simulation: Steady-State vs CCDE vs Capped CCDE
clear; close all;

% 1. Define True Plant Parameters (Full 2nd-Order System)
R = 6.41;     % Resistance (Ohm)
L = 7.7e-3;   % Inductance (Henry)
Kt = 0.011;   % Torque constant (Nm/A)
Kb = 0.011;   % Back-EMF constant (V/(rad/s))
J = 5.46e-6;  % Inertia (Kg m^2)
D = 6.52e-7;  % Friction (Nm/(rad/s))

Ts = 0.03;   % Sample time (s)
t = 0:Ts:0.5; % Simulation time vector
N = length(t);
V_max = 6.0;  % Maximum voltage limit (V)

% 2. Define Reference Trajectory (Step to 50 rad/s at t = 0.05s)
w_ref = zeros(N, 1);
w_ref(t >= 0.05) = 50;

% 3. Steady-State Feedforward Mapping
v_ss = ((R * D) / Kt + Kb) * w_ref;

% 4. CCDE 1st-Order Inverse Mapping
num_1st = Kt;
den_1st = [J*R, D*R + Kt*Kb];
Gz_1st = c2d(tf(num_1st, den_1st), Ts, 'zoh');
[num_cell, den_cell] = tfdata(Gz_1st, 'v');
a1 = den_cell(2);
b1 = num_cell(2);

v_ccde = zeros(N, 1);
for k = 1:N-1
    % Inverse equation: v_k = (1/b1)*w_{ref,k+1} + (a1/b1)*w_{ref,k}
    v_ccde(k) = (1/b1)*w_ref(k+1) + (a1/b1)*w_ref(k);
end
v_ccde(N) = v_ccde(N-1); % Hold last value

% 5. Apply Hard Saturation (Capped CCDE)
v_ccde_capped = max(min(v_ccde, V_max), -V_max);

% 6. Simulate the Actual 2nd-Order Plant
Gs_true = tf(Kt, [J*L, D*L + J*R, D*R + Kt*Kb]);
Gz_true = c2d(Gs_true, Ts, 'zoh');

w_actual_ss = lsim(Gz_true, v_ss, t);
w_actual_ccde = lsim(Gz_true, v_ccde, t);
w_actual_ccde_capped = lsim(Gz_true, v_ccde_capped, t);

% 7. Plot Results
figure('Name', 'Feedforward Control Comparison', 'Position', [100, 100, 800, 600]);

% Velocity Plot
subplot(2,1,1);
stairs(t, w_ref, 'k--', 'LineWidth', 1.5); hold on;
plot(t, w_actual_ss, 'b', 'LineWidth', 1.5);
plot(t, w_actual_ccde, 'r', 'LineWidth', 1.5);
plot(t, w_actual_ccde_capped, 'g', 'LineWidth', 1.5);
title('Velocity Tracking Comparison');
ylabel('Velocity (rad/s)');
legend('Target Velocity', 'Steady-State', 'CCDE (Uncapped)', 'CCDE (Capped)', 'Location', 'southeast');
grid on;

% Voltage Plot
subplot(2,1,2);
stairs(t, v_ss, 'b', 'LineWidth', 1.5); hold on;
stairs(t, v_ccde, 'r', 'LineWidth', 1.5);
stairs(t, v_ccde_capped, 'g--', 'LineWidth', 1.5);
title('Control Input (Voltage)');
xlabel('Time (s)');
ylabel('Voltage (V)');
legend('Steady-State', 'CCDE (Uncapped)', 'CCDE (Capped)', 'Location', 'northeast');
grid on;