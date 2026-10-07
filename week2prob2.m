% Define system parameters
R = 1;        % Wire resistance (Ohm)
L = 0.001;    % Wire inductance (Henry)
Kt = 0.01;    % Motor torque constant (Nm/A)
Kb = 0.01;    % Back-EMF constant (V/(rad/s))
D = 1e-5;     % Friction (Nm/(rad/s))
J = 1e-5;     % Motor inertia (kg*m^2)

% Define the base transfer function G(s)
% G(s) = Kt / (J*L*s^2 + (D*L + J*R)*s + (D*R + Kt*Kb))
num_G = [Kt];
den_G = [J*L, (D*L + J*R), (D*R + Kt*Kb)];
Gs = tf(num_G, den_G);

% Introduce a 10ms (0.01s) delay using a 2nd order Pade approximation
delay_time = 0.01;
[num_delay, den_delay] = pade(delay_time, 2);
Gd = tf(num_delay, den_delay);

% Calculate the delayed transfer function
Gs_delayed = Gs * Gd;

% Generate a Bode diagram for comparison
figure;
hold off;
bode(Gs);
hold on;
bode(Gs_delayed);
grid on;
legend('no delay', '10ms delay');