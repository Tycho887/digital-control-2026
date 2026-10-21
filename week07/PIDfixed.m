%% Improved PID Tuning and Stable CCDE Simulation
z = tf('z', Ts);

% 1. Include the 1-sample feedback delay (detect_delay in Simulink) during tuning
plant_delayed = plant * z^(-1);

% 2. Construct a discrete PID template with BackwardEuler filter to guarantee 0 < z_d < 1
%    (Alternatively, use 'PI' instead of C_template to eliminate derivative kick entirely)
C_template = pid(1, 1, 1, 0.02, Ts, ...
    'IFormula', 'ForwardEuler', ...
    'DFormula', 'BackwardEuler');

% Target a crossover frequency wc = 25 rad/s with 65 deg phase margin
% This forces Ki high enough to eliminate the >1000 ms slow mechanical tail
wc = 25; 
opts = pidtuneOptions('PhaseMargin', 65);
[C, info] = pidtune(plant_delayed, C_template, wc, opts);
disp(C); disp(info);

% Extract CCDE coefficients (ensure vectors have length 3 even if D = 0)
C_z = tf(C);
[num, den] = tfdata(C_z, 'v');
num = [num, zeros(1, 3 - length(num))];
den = [den, zeros(1, 3 - length(den))];

b0 = num(1); b1 = num(2); b2 = num(3);
a1 = den(2); a2 = den(3);

fprintf('u[k] = %.6f*e[k] + %.6f*e[k-1] + %.6f*e[k-2] - (%.6f*u[k-1] + %.6f*u[k-2])\n', ...
    b0, b1, b2, a1, a2);

%% 3. Linear Step Response Verification
t_sim = 0:Ts:0.6;
sys_cl = feedback(C * plant, z^(-1));
opt = stepDataOptions('StepAmplitude', referenceSpeed);
[y_lin, t_lin] = step(sys_cl, t_sim, opt);

%% 4. CCDE Step Response Loop (Without Derivative-Saturation Ringing)
[numP, denP] = tfdata(plant, 'v');
q1 = numP(2); q2 = numP(3);
p1 = denP(2); p2 = denP(3);

N_steps = length(t_sim);
r_vel = zeros(1, N_steps);
r_vel(t_sim >= stepTime) = referenceSpeed;
r_mot = r_vel * (gear / wheelRadius);

e = zeros(1, N_steps);
u = zeros(1, N_steps);        % Controller internal CCDE state
u_act = zeros(1, N_steps);    % Saturated actuator voltage applied to motor
y_mot = zeros(1, N_steps);

for k = 3:N_steps
    % Discrete motor plant driven by saturated actuator voltage u_act
    y_mot(k) = -p1*y_mot(k-1) - p2*y_mot(k-2) + q1*u_act(k-1) + q2*u_act(k-2);

    % Error with 1-sample feedback delay y_mot(k-1) matching detect_delay
    e(k) = r_mot(k) - y_mot(k-1);

    % Evaluate linear CCDE
    u(k) = b0*e(k) + b1*e(k-1) + b2*e(k-2) - (a1*u(k-1) + a2*u(k-2));

    % Saturate actuator output sent to the plant
    u_act(k) = min(max(u(k), -voltageLimit), voltageLimit);
end

y_wheel = y_mot * (wheelRadius / gear);

%% Plot Comparison
figure('Name', 'Stabilized Discrete PID Step Response');
subplot(2, 1, 1);
stairs(t_sim, r_vel, 'k--', 'LineWidth', 1.2); hold on;
stairs(t_lin + stepTime, y_lin, 'b-', 'LineWidth', 1.5);
stairs(t_sim, y_wheel, 'r:', 'LineWidth', 1.8);
xlim([0, 0.6]); grid on;
ylabel('Wheel Velocity (m/s)');
title('Step Response (wc = 25 rad/s, BackwardEuler Filter)');
legend('Reference', 'Linear Closed-Loop', 'CCDE Loop', 'Location', 'best');

subplot(2, 1, 2);
stairs(t_sim, u_act, 'm-', 'LineWidth', 1.5);
yline(voltageLimit, 'k--'); yline(-voltageLimit, 'k--');
xlim([0, 0.6]); ylim([-voltageLimit-1, voltageLimit+1]); grid on;
xlabel('Time (s)'); ylabel('Motor Voltage (V)');
title('Actuator Control Effort');