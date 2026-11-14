%% ========================================================================
%  kalman_filter_tuning/solve_dare.m
%
%  Stationary gain bank for the planar differential-drive robot.
%
%  State:  x = [ v ; omega ; b_a ]
%      v      forward velocity                  [m/s]
%      omega  yaw rate                          [rad/s]
%      b_a    residual accelerometer bias       [m/s^2]
%
%  Heading theta and distance s/x/y are NOT states. The firmware integrates
%  them outside the filter with the trapezoidal rule:
%      theta += 0.5*(omega_prev + omega)*Ts
%      s     += 0.5*(v_prev + v)*Ts
%
%  The DARE is solved per observable block at the rate each measurement is
%  actually applied:
%      [v, b_a]  1 kHz predict, 100 Hz wheel-v update
%      omega     1 kHz gyro update
%      omega     100 Hz wheel-omega update
%
%  Noise inputs are the 1-sigma values recorded in the `cal upside` logs
%  (src/logs/device-monitor-261114-152906.log and the 14:40 capture):
%      gyro yaw     0.08-0.17 deg/s
%      accel fwd    0.031-0.123 m/s^2
%      wheel v      0.013-0.0145 m/s (10 ms encoder window)
%      wheel omega  0.18 rad/s        (10 ms encoder window)
%
%  q_v and q_om are disturbance inputs, not sensor noise: q_v covers
%  unmodelled velocity disturbances and q_om matches the 0.7 rad/s^2 turn
%  limiter. q_ba is a placeholder until an Allan-variance capture exists.
%
%  MATLAB R2016b+ (local functions in scripts) and the Control System
%  Toolbox (`dare`) are required.
% ========================================================================

clear; close all; clc;

%% ---------------- 1. PARAMETERS -----------------------------------------
Ts          = 1e-3;   % filter period [s]
wheel_div   = 10;     % wheel update every 10 filter steps (100 Hz)
Ts_wheel    = Ts * wheel_div;

% Log-derived 1-sigma values.
sigma_gyro_dps    = 0.17;    % [deg/s]
sigma_accel       = 0.12;    % [m/s^2], in-motion
sigma_wheel_v     = 0.0145;  % [m/s], 48-PPR encoder fallback, 10 ms
sigma_wheel_omega = 0.18;    % [rad/s], encoder fallback, 10 ms

% Disturbance / random-walk seeds.
q_v  = 1.0;     % [m/s^2] unmodelled velocity disturbance
q_om = 0.7;     % [rad/s^2] unmodelled angular acceleration (turn limiter)
q_ba = 3e-3;    % [m/s^2/sqrt(s)] accelerometer bias random walk

d2r         = pi/180;
sigma_gyro  = sigma_gyro_dps * d2r;   % [rad/s]

%% ---------------- 2. TRANSLATION BLOCK [v, b_a] -------------------------
V = 1; BA = 2;

F_t = [1, -Ts;
       0,  1];
B_t = [Ts; 0];

% Accelerometer input noise plus unmodelled velocity disturbance, then the
% bias random walk.
Q_t = B_t * (sigma_accel^2 + q_v^2) * B_t' + diag([0, q_ba^2 * Ts]);

% Lift the 1 kHz model to the 10 ms wheel interval.
F_tw = F_t^wheel_div;
Q_tw = zeros(2);
Ak = eye(2);
for k = 1:wheel_div
    Q_tw = Q_tw + Ak * Q_t * Ak';
    Ak = Ak * F_t;
end

H_v = [1, 0];
R_v = sigma_wheel_v^2;

[P_v, ~, ~] = dare(F_tw', H_v', Q_tw, R_v);
K_wheel_v = P_v * H_v' / (H_v * P_v * H_v' + R_v);   % 2x1 over [v, b_a]

%% ---------------- 3. ROTATION BLOCK, GYRO AT 1 kHz ----------------------
F_r = 1;
Q_g = (q_om * Ts)^2;
R_g = sigma_gyro^2;

[P_g, ~, ~] = dare(F_r', 1, Q_g, R_g);
K_gyro = P_g / (P_g + R_g);                         % scalar over omega

%% ---------------- 4. ROTATION BLOCK, WHEEL OMEGA AT 100 Hz ---------------
Q_w = wheel_div * (q_om * Ts)^2;
R_w = sigma_wheel_omega^2;

[P_w, ~, ~] = dare(F_r', 1, Q_w, R_w);
K_wheel_omega = P_w / (P_w + R_w);                  % scalar over omega

%% ---------------- 5. OBSERVABILITY AND MULTI-RATE CHECK ----------------
F = [1,  0, -Ts;
     0,  1,   0;
     0,  0,   1];   % state order [v, omega, b_a]
H = [1,  0,   0;    % wheel v
     0,  1,   0];   % wheel omega (same row as gyro)

rank_O = observability_rank(F, H);
cycle_eig = multirate_cycle(F, K_wheel_v, K_gyro, K_wheel_omega, wheel_div);

%% ---------------- 6. REPORT ---------------------------------------------
fprintf('\n=== 3-state stationary DARE ===\n');
fprintf('Ts = %.3f ms, wheel update every %d steps (%.1f Hz)\n', ...
        Ts*1e3, wheel_div, 1/Ts_wheel);
fprintf('sigma_gyro = %.3f deg/s, sigma_accel = %.3f m/s^2\n', ...
        sigma_gyro_dps, sigma_accel);
fprintf('sigma_wheel_v = %.4g m/s, sigma_wheel_omega = %.4g rad/s\n', ...
        sigma_wheel_v, sigma_wheel_omega);
fprintf('q_v = %.3g m/s^2, q_om = %.3g rad/s^2, q_ba = %.3g m/s^2/sqrt(s)\n', ...
        q_v, q_om, q_ba);

fprintf('\nObservability rank of [v, omega, b_a]: %d / 3\n', rank_O);
if rank_O < 3
    warning('The state is not fully observable with these rows.');
end

fprintf('\nGain bank for src/src/kalman.h:\n');
fprintf('  K_WHEEL_V     = { %.9ff, %.9ff }\n', ...
        K_wheel_v(1), K_wheel_v(2));
fprintf('  K_GYRO        = %.9ff\n', K_gyro);
fprintf('  K_WHEEL_OMEGA = %.9ff\n', K_wheel_omega);

fprintf('\nsqrt(diag(P)): v %.4g m/s, b_a %.4g m/s^2, omega %.4g rad/s\n', ...
        sqrt(P_v(1,1)), sqrt(P_v(2,2)), sqrt(P_g));
fprintf('multi-rate cycle max |eig| = %.6f\n', cycle_eig);
if cycle_eig >= 1
    warning('The multi-rate cycle is not stable with this gain bank.');
end

%% ---------------- 7. SAVE ----------------------------------------------
save('dare_results.mat', 'Ts', 'wheel_div', 'F', 'H', 'K_wheel_v', ...
     'K_gyro', 'K_wheel_omega', 'P_v', 'P_g', 'P_w', 'sigma_gyro', ...
     'sigma_accel', 'sigma_wheel_v', 'sigma_wheel_omega', 'q_v', 'q_om', ...
     'q_ba');
write_csv('dare_gains.csv', K_wheel_v, K_gyro, K_wheel_omega);
write_header('dare_gains.h', K_wheel_v, K_gyro, K_wheel_omega);
fprintf('\nSaved dare_results.mat, dare_gains.csv and dare_gains.h\n');

%% ========================================================================
%  Local functions
%% ========================================================================

function r = observability_rank(F, H)
    n = size(F, 1);
    O = H;
    A = H;
    for k = 1:n-1
        A = A * F;
        O = [O; A];
    end
    r = rank(O);
end

function max_eig = multirate_cycle(F, K_v, K_g, K_w, wheel_div)
    n = size(F, 1);
    H_v = [1, 0, 0];
    H_g = [0, 1, 0];
    H_w = [0, 1, 0];
    K_v_vec = [K_v(1); 0; K_v(2)];
    K_g_vec = [0; K_g; 0];
    K_w_vec = [0; K_w; 0];

    M = eye(n);
    for k = 0:99
        M = F * M;                                  % predict
        M = (eye(n) - K_g_vec * H_g) * M;           % gyro every step
        if mod(k, wheel_div) == 0
            M = (eye(n) - K_v_vec * H_v) * M;       % wheel v
            M = (eye(n) - K_w_vec * H_w) * M;       % wheel omega
        end
    end
    max_eig = max(abs(eig(M)));
end

function write_csv(filename, K_v, K_g, K_w)
    fid = fopen(filename, 'w');
    if fid < 0
        warning('Could not open %s for writing.', filename);
        return;
    end
    fprintf(fid, 'gain,value\n');
    fprintf(fid, 'K_WHEEL_V0,%.12g\n', K_v(1));
    fprintf(fid, 'K_WHEEL_V1,%.12g\n', K_v(2));
    fprintf(fid, 'K_GYRO,%.12g\n', K_g);
    fprintf(fid, 'K_WHEEL_OMEGA,%.12g\n', K_w);
    fclose(fid);
end

function write_header(filename, K_v, K_g, K_w)
    fid = fopen(filename, 'w');
    if fid < 0
        warning('Could not open %s for writing.', filename);
        return;
    end
    fprintf(fid, '// Generated by kalman_filter_tuning/solve_dare.m\n');
    fprintf(fid, '// Copy into the kf_gains namespace in src/src/kalman.h.\n');
    fprintf(fid, 'constexpr float K_WHEEL_V[2] = {%.9ff, %.9ff};\n', ...
            K_v(1), K_v(2));
    fprintf(fid, 'constexpr float K_GYRO = %.9ff;\n', K_g);
    fprintf(fid, 'constexpr float K_WHEEL_OMEGA = %.9ff;\n', K_w);
    fclose(fid);
end
