% Define system parameters
R = 1;        % Wire resistance (Ohm)
L = 0.001;    % Wire inductance (Henry)
Kt = 0.01;    % Motor torque constant (Nm/A)
Kb = 0.01;    % Back-EMF constant (V/(rad/s))
D = 1e-5;     % Friction (Nm/(rad/s))
J = 1e-5;     % Motor inertia (kg*m^2)

% Define the base transfer function G(s)
num_G = [Kt];
den_G = [J*L, (D*L + J*R), (D*R + Kt*Kb)];
Gs = tf(num_G, den_G);

% Introduce a 10ms (0.01s) delay using a 2nd order Pade approximation
delay_time = 0.01;
[num_delay, den_delay] = pade(delay_time, 2);
Gd = tf(num_delay, den_delay);

% Calculate the total delayed open-loop transfer function
Gs_delayed = Gs * Gd;

% Tune the PI controller using the automated pidtune function
% The function assumes a negative unity feedback loop
C_pi = pidtune(Gs_delayed, 'PI');

% Display the resulting controller parameters (Kp and Ki)
disp('Tuned PI Controller:');
disp(C_pi);

% Construct the closed-loop system 
% feedback(forward_path, feedback_path)
sys_cl = feedback(C_pi * Gs_delayed, 1);

% Evaluate the system with a step response
figure;
step(sys_cl);
title('Closed-Loop Step Response (PI Controller with 10ms Delay)');
grid on;

% Extract Kp and Ki directly from the tuned controller object
Kp_tuned = C_pi.Kp;
Ki_tuned = C_pi.Ki;

% Calculate the integrator time constant (tau_i)
tau_i_tuned = Kp_tuned / Ki_tuned;

% Display the specific parameters
fprintf('Proportional Gain (Kp): %f\n', Kp_tuned);
fprintf('Integrator Time Constant (tau_i): %f seconds\n', tau_i_tuned);

% Gs_delayed is the total open-loop transfer function (Motor + Delay)

% Target phase margin
target_pm = 60; 
target_phase = -180 + target_pm;

% Extract Bode plot data over a suitable frequency range
[mag, phase, w] = bode(Gs_delayed);

% Squeeze arrays to remove singleton dimensions
mag = squeeze(mag);
phase = squeeze(phase);

% Find the index where the phase crosses the target phase (-120 degrees)
% Interpolation could be used for higher precision
[~, index] = min(abs(phase - target_phase));

% Extract the crossover frequency and magnitude at that exact point
w_c = w(index);
mag_uncompensated_linear = mag(index);
mag_uncompensated_dB = 20 * log10(mag_uncompensated_linear);

% Calculate the required proportional gain Kp
Kp_tuned = 10^(-mag_uncompensated_dB / 20);

fprintf('Target Phase Angle: %f degrees\n', target_phase);
fprintf('Crossover Frequency (w_c): %f rad/s\n', w_c);
fprintf('Uncompensated Magnitude: %f dB\n', mag_uncompensated_dB);
fprintf('Calculated Proportional Gain (Kp): %f\n', Kp_tuned);