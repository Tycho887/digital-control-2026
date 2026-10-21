% Left Motor parameters
L = 7.7e-3;  % Inductance (Henry)
R = 6.41;     % Resistance (Ohm)
Kt = 0.011; % Motor torque constant (Nm/A)
Kb = 0.011; % Back-EMF constant (V/(rad/s))
J = 5.46e-6;  % Inertia (Kg m^2)
D = 6.52e-7;  % Friction (Nm/(rad/s))
% Right Motor parameters
% L = 3.3e-4
% R = 2.86
% Kb = 0.00798
% Kt = 0.00798
% J = 5.46e-6
% D = 3.38e-6

% Continuous transfer function G(s)
% G(s) = Kt / (J*L*s^2 + (D*L + J*R)*s + D*R + Kt*Kb)
num = Kt;
den = [J*L, D*L + J*R, D*R + Kt*Kb];
Gs = tf(num, den);

% Discretize the system
Ts = 0.01; % Sample time in seconds
Gz = c2d(Gs, Ts, 'zoh');

% Extract poles (roots of the characteristic equation)
discrete_poles = pole(Gz);
pole_magnitudes = abs(discrete_poles);

% Check if all roots are strictly inside the unit circle
is_stable = all(pole_magnitudes < 1);

% Display results
disp('Discrete Poles:');
disp(discrete_poles);
disp('Pole Magnitudes:');
disp(pole_magnitudes);

if is_stable
    disp('System is stable (all roots inside the unit circle).');
else
    disp('System is unstable (roots on or outside the unit circle).');
end

% Assume Gs (continuous transfer function) and Ts (sample time) are defined
Gz = c2d(Gs, Ts, 'zoh');

% Extract numerator and denominator coefficients
% tfdata returns cell arrays, so curly braces extract the arrays
[num_cell, den_cell] = tfdata(Gz, 'v');

% A standard 2nd-order discrete transfer function takes the form:
% den = [1, a1, a2], num = [b0, b1, b2]
a1 = den_cell(2);
a2 = den_cell(3);

b0 = num_cell(1);
b1 = num_cell(2);
b2 = num_cell(3);

% Ensure coefficient vectors contain the expected second-order terms
num_cell = [zeros(1, 3 - numel(num_cell)), num_cell];
den_cell = [zeros(1, 3 - numel(den_cell)), den_cell];

% Note: If the discretization method produces a strict proper system 
% (b0 = 0), the control sequence must be shifted forward by one step, 
% using b1 as the leading term and tracking a future velocity reference.

% 1. Forward Transition Operator (Shift Register for Velocity)
A_forward = [-a1, -a2; 
    1,   0];

% 2. Inverse Control Transition Operator (Shift Register for Voltage)
% Pre-inverted by dividing by the leading coefficient b0
A_control = [-b1/b0, -b2/b0;
    1,      0];

% Matrix for incorporating the velocity trajectory terms into the control law
B_control = [1/b0, a1/b0, a2/b0;
    0,    0,     0];

disp('Forward Transition Operator:');
disp(A_forward);

disp('Control Transition Operator:');
disp(A_control);