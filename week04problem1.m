% MATLAB script to visualize step responses and pole/zero effects
% 1. Define sample time for discrete system (unspecified)
Ts = -1; 

% ---------------------------------------------------------
% Figure 1: Effect of Zero Location on Overshoot
% ---------------------------------------------------------
figure('Name', 'Effect of Zero Location');
hold on;

% Base denominator parameters (equivalent s-plane damping ratio = 0.5)
r = 0.8;
theta = deg2rad(18);
den = [1, -2*r*cos(theta), r^2];

% Test different zero locations moving towards +1
zero_locations = [0.6, 0.7, 0.8, 0.9];

for idx = 1:length(zero_locations)
    z_loc = zero_locations(idx);
    
    % Define Numerator: z - z_loc
    num = [1, -z_loc]; 
    sysD = tf(num, den, Ts);
    
    % Normalize gain K so steady-state output equals the step size
    K = 1 / dcgain(sysD);
    sysD_normalized = K * sysD;
    
    % Compute and plot discrete step response
    step(sysD_normalized);
end

title('Effect of Zero Location on Overshoot');
legend('z = 0.6', 'z = 0.7', 'z = 0.8', 'z = 0.9', 'Location', 'best');
grid on;
hold off;

% ---------------------------------------------------------
% Figure 2: Effect of an Extra Pole on Rise Time
% ---------------------------------------------------------
figure('Name', 'Effect of Extra Pole Location');
hold on;

% Fixed numerator for third-order pole test
num_pole_test = [1, -0.8*cos(theta)];

% Test different locations for a third pole moving towards +1
extra_poles = [-0.9, 0.2, 0.9];

for idx = 1:length(extra_poles)
    p_ext = extra_poles(idx);
    
    % Convolve base second-order denominator with extra pole (z - p_ext)
    den_ext = conv(den, [1, -p_ext]); 
    sysD_pole = tf(num_pole_test, den_ext, Ts);
    
    % Normalize gain K so steady-state output equals the step size
    K_pole = 1 / dcgain(sysD_pole);
    sysD_pole_normalized = K_pole * sysD_pole;
    
    % Compute and plot discrete step response
    step(sysD_pole_normalized);
end

title('Effect of Extra Pole Location on Rise Time');
legend('Pole at -0.9', 'Pole at 0.2', 'Pole at 0.9', 'Location', 'best');
grid on;
hold off; 