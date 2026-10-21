%% PID tuning for the week07 wheel model.
% Motor coefficients identified in week06 (see week06/basebot_motor_parameters.csv,
% left wheel; swap in the right-wheel row for the right motor).
% Run before simulating week07_wheel_model.
clear; clc;

%% Hard-coded motor coefficients and sample time
L = 0.00768482381376416;    % H       armature inductance
R = 6.41139804096171;       % Ohm     armature resistance
Kt = 0.010293561274142;     % Nm/A    torque constant
Kb = Kt;                    % V/(rad/s) back-EMF constant
J = 5.46186035789905e-06;   % kg m^2  rotor + reflected load inertia
D = 6.52401278374518e-07;   % Nm/(rad/s) viscous friction
gear = 9.68;                % gear ratio
wheelRadius = 0.03;         % m
Ts = 0.005;                 % s       controller sample time
voltageLimit = 8;           % V
referenceSpeed = 0.1;       % m/s
stepTime = 0.2;             % s

%% Load the model and push parameters into its workspace
model = 'week07_wheel_model';
load_system(fullfile(fileparts(mfilename('fullpath')), [model '.slx']));
workspace = get_param(model, 'ModelWorkspace');

%% Motor speed plant, discretized with the zero-order hold of the drive output
plant = tf(Kt, [L*J, L*D + R*J, R*D + Kt*Kb]);   % rad/s per V
plant = c2d(plant, Ts, 'zoh');

%% Standard PID tuning (pass a crossover frequency as 3rd arg to target bandwidth)
[C, info] = pidtune(plant, 'PID');
disp(C); disp(info);

%% Feed the result back into the model (mask uses wheel.Kp)
wheel = struct('L', L, 'R', R, 'Kt', Kt, 'Kb', Kb, 'J', J, 'D', D, ...
    'gear', gear, 'wheelRadius', wheelRadius, 'Kp', C.Kp, 'Ts', Ts, ...
    'voltageLimit', voltageLimit);
assignin(workspace, 'wheel', wheel);
assignin(workspace, 'referenceSpeed', referenceSpeed);
assignin(workspace, 'stepTime', stepTime);

% Convert the tuned PID object into a discrete transfer function
C_z = tf(C);

% Extract numerator and denominator polynomial vectors
[num, den] = tfdata(C_z, 'v');

% Display CCDE coefficients
b0 = num(1);
b1 = num(2);
b2 = num(3);
a1 = den(2);
a2 = den(3);

fprintf('u[k] = %.6f*e[k] + %.6f*e[k-1] + %.6f*e[k-2] - (%.6f*u[k-1] + %.6f*u[k-2])\n', ...
    b0, b1, b2, a1, a2);

%% 1. Linear Closed-Loop Step Response (Transfer Function)
t_sim = 0:Ts:0.6;
z = tf('z', Ts);

% Closed-loop system (with and without the 1-sample feedback delay from Simulink)
sys_cl = feedback(C * plant, 1);
sys_cl_delayed = feedback(C * plant, z^(-1));

% Step options matching stepTime and referenceSpeed
opt = stepDataOptions('StepAmplitude', referenceSpeed);
[y_lin, t_lin] = step(sys_cl, t_sim, opt);
[y_del, ~]     = step(sys_cl_delayed, t_sim, opt);

%% 2. Discrete CCDE Step Response (Difference Equation Loop)
% Extract discrete plant coefficients: y_m[k] = -p1*y_m[k-1] - p2*y_m[k-2] + q1*u[k-1] + q2*u[k-2]
[numP, denP] = tfdata(plant, 'v');
q1 = numP(2); 
q2 = numP(3);
p1 = denP(2); 
p2 = denP(3);

N_steps = length(t_sim);
r_vel   = zeros(1, N_steps);
r_vel(t_sim >= stepTime) = referenceSpeed;  % Step reference (m/s)

% Convert wheel reference (m/s) to motor shaft reference (rad/s)
r_mot = r_vel * (gear / wheelRadius);

e = zeros(1, N_steps);      % Motor speed error (rad/s)
u = zeros(1, N_steps);      % Controller output voltage (V)
y_mot = zeros(1, N_steps);  % Motor angular velocity (rad/s)

for k = 3:N_steps
    % Update motor speed from discrete plant model
    y_mot(k) = -p1*y_mot(k-1) - p2*y_mot(k-2) + q1*u(k-1) + q2*u(k-2);

    % Tracking error (using y_mot(k) for standard feedback, or y_mot(k-1) for 1-sample delay)
    e(k) = r_mot(k) - y_mot(k);

    % Discrete PID difference equation (CCDE)
    u_raw = b0*e(k) + b1*e(k-1) + b2*e(k-2) - (a1*u(k-1) + a2*u(k-2));

    % Apply actuator voltage saturation
    u(k) = min(max(u_raw, -voltageLimit), voltageLimit);
end

% Convert motor angular velocity (rad/s) back to wheel linear velocity (m/s)
y_wheel = y_mot * (wheelRadius / gear);

%% Plot Linear and CCDE Step Responses
figure('Name', 'Discrete PID Step Response');
subplot(2, 1, 1);
stairs(t_sim, r_vel, 'k--', 'LineWidth', 1.2); hold on;
stairs(t_lin + stepTime, y_lin, 'b-', 'LineWidth', 1.5);
stairs(t_lin + stepTime, y_del, 'g-.', 'LineWidth', 1.2);
stairs(t_sim, y_wheel, 'r:', 'LineWidth', 1.8);
xlim([0, 0.6]);
grid on;
ylabel('Wheel Velocity (m/s)');
title('Discrete PID Controller Step Response');
legend('Reference', 'Linear Closed-Loop', 'Linear (with 1-Ts Delay)', 'CCDE Loop (Saturated)', 'Location', 'best');

subplot(2, 1, 2);
stairs(t_sim, u, 'm-', 'LineWidth', 1.5);
yline(voltageLimit, 'k--');
yline(-voltageLimit, 'k--');
xlim([0, 0.6]);
grid on;
xlabel('Time (s)');
ylabel('Control Effort u[k] (V)');
title('Controller Output Voltage');