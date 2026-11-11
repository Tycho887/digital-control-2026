%% ========================================================================
%  STATIC KALMAN FILTER - IMU + wheel-encoder fusion (planar diff. drive)
%  ----------------------------------------------------------------------
%  Solves the stationary (predictor) DARE for the constant gain K_inf.
%
%  State  x = [ v ; omega ; b_a ; b_g ]
%  Input  u = [ a_m ; omega_m ]          (MPU-9250 readings)
%  Meas.  z = [ v_enc ; omega_enc ]      (wheel odometry)
%
%  All datasheet values are SEED values - refine from your own logs.
%  ========================================================================

clear; close all; clc;

%% ---------------- 1. SENSOR NOISE PARAMETERS (datasheet seeds) ----------

% ----- MPU-9250  (AFS_SEL=1: +/-4g, FS_SEL=2: +/-1000 dps, DLPFCFG=2) ----
gyro_noise_density   = 0.01;         % [deg/s/sqrt(Hz)]   rate-noise spectral density
accel_noise_density  = 300e-6;       % [g/sqrt(Hz)]       noise power spectral density
gyro_bias_init_std   = 0.05 * 1000;  % [deg/s]            5% of FS (initial ZRO tol)
accel_bias_init_std  = 60e-3;        % [g]                XY spec, conservative
gyro_random_walk     = 0.001;        % [deg/s/sqrt(s)]    ***PLACEHOLDER*** (Allan dev)
accel_random_walk    = 1e-4;         % [g/sqrt(s)]        ***PLACEHOLDER*** (Allan dev)
DLPF_BW              = 92;           % [Hz]               DLPFCFG=2

% ----- Wheel encoders  (unspecified A/B, 48 edges / motor rev) -----------
% Resolution alone does NOT determine velocity uncertainty - timing jitter,
% quantisation, wheel imperfection and slip dominate.  REPLACE with data.
wheel_radius          = 0.12;        % [m]     ***PLACEHOLDER*** (BOM)
gear_ratio            = 1;           % [-]     ***PLACEHOLDER*** (BOM)
encoder_edges_per_rev = 48;          % [edges/rev] motor-side
wheel_velocity_sigma  = 0.02;        % [m/s]   ***PLACEHOLDER***
yaw_rate_sigma        = 0.05;        % [rad/s] ***PLACEHOLDER***

%% ---------------- 2. UNIT CONVERSIONS -----------------------------------
d2r    = pi/180;
g2mps2 = 9.80665;

gyro_noise_density   = gyro_noise_density  * d2r;        % [rad/s/sqrt(Hz)]
accel_noise_density  = accel_noise_density * g2mps2;     % [m/s^2/sqrt(Hz)]
gyro_bias_init_std   = gyro_bias_init_std  * d2r;        % [rad/s]
accel_bias_init_std  = accel_bias_init_std * g2mps2;     % [m/s^2]
gyro_random_walk     = gyro_random_walk    * d2r;        % [rad/s/sqrt(s)]
accel_random_walk    = accel_random_walk   * g2mps2;     % [m/s^2/sqrt(s)]

%% ---------------- 3. DISCRETE-TIME MODEL --------------------------------
Ts = 0.001;   % [s] filter period = 200 Hz

% State matrix A = I + A_c*Ts (exact for this integrator form)
A = [ 1  0  -Ts  0 ;
      0  1   0  -Ts ;
      0  0   1   0 ;
      0  0   0   1 ];

% Input matrix (IMU as control input)
B = [ Ts  0 ;
      0  Ts ;
      0   0 ;
      0   0 ];

% Measurement matrix
H = [ 1 0 0 0 ;     % v_enc      -> v
      0 1 0 0 ];    % omega_enc  -> omega

n = size(A,1); m = size(H,1);
assert(isequal(size(A),[n n]) && isequal(size(H),[m n]), 'dimension mismatch');

%% ---------------- 4. PROCESS NOISE Q ------------------------------------
% IMU noise enters Q because the IMU is the control input.  Integrated
% per-step variance of velocity/omega:
%     sigma_step  = noise_density * sqrt(BW)      [units]
%     Q_v         = (sigma_step * Ts)^2           [units^2]
q_v     = accel_noise_density^2 * DLPF_BW * Ts^2;    % [(m/s)^2]   per step
q_omega = gyro_noise_density^2  * DLPF_BW * Ts^2;    % [(rad/s)^2] per step

% Bias random-walk contributions
q_ba = accel_random_walk^2 * Ts;                     % [(m/s^2)^2] per step
q_bg = gyro_random_walk^2  * Ts;                     % [(rad/s)^2] per step

Q = diag([ q_v, q_omega, q_ba, q_bg ]);

%% ---------------- 5. MEASUREMENT NOISE R --------------------------------
R = diag([ wheel_velocity_sigma^2 , ...
           yaw_rate_sigma^2 ]);

%% ---------------- 6. INITIAL COVARIANCE (for startup transient) ---------
P0 = diag([ 0.5^2 , ...                  % v     : 0.5 m/s
            (30*d2r)^2 , ...             % omega : 30 deg/s
            accel_bias_init_std^2 , ...  % b_a
            gyro_bias_init_std^2 ]);     % b_g

%% ---------------- 7. SOLVE STATIONARY DARE ------------------------------
% Kalman (predictor) DARE:
%     P = A P A' - A P H'(H P H' + R)^-1 H P A' + Q
%
% MATLAB dare(A,B,Q,R) solves:
%     X = A' X A - A' X B(B' X B + R)^-1 B' X A + Q
%
% By duality (A -> A', B -> H') we recover exactly the Kalman DARE.
% P_inf returned is the a-priori error covariance P^-.
[P_inf, ~, ~] = dare(A', H', Q, R);

% Stationary gain (measurement-update form)
K_inf = P_inf * H' / (H * P_inf * H' + R);

% Closed-loop (predictor) system and eigenvalues
A_cl   = A - A * K_inf * H;
eigs_cl = eig(A_cl);

%% ---------------- 8. REPORT --------------------------------------------
fprintf('\n=== Stationary Kalman filter ===\n');
fprintf('Ts = %.1f ms,   IMU DLPF BW = %d Hz\n', Ts*1000, DLPF_BW);

fprintf('\nProcess noise Q (diag):\n');
fprintf('  q_v     = %.3e  [(m/s)^2/step]\n'    , Q(1,1));
fprintf('  q_omega = %.3e  [(rad/s)^2/step]\n'  , Q(2,2));
fprintf('  q_ba    = %.3e  [(m/s^2)^2/step]\n'  , Q(3,3));
fprintf('  q_bg    = %.3e  [(rad/s)^2/step]\n'  , Q(4,4));

fprintf('\nMeasurement noise R (std):\n');
fprintf('  sigma_v_enc     = %.4f [m/s]\n'   , sqrt(R(1,1)));
fprintf('  sigma_omega_enc = %.4f [rad/s]\n' , sqrt(R(2,2)));

fprintf('\nA-priori covariance P^- (std):\n');
P_std = sqrt(diag(P_inf));
fprintf('  sigma_v     = %.4e [m/s]\n'  , P_std(1));
fprintf('  sigma_omega = %.4e [rad/s]\n', P_std(2));
fprintf('  sigma_b_a   = %.4e [m/s^2]\n', P_std(3));
fprintf('  sigma_b_g   = %.4e [rad/s]\n', P_std(4));

fprintf('\nStationary gain K_inf:\n');  disp(K_inf);

fprintf('Closed-loop |eig(A - A*K*H)|:\n  ');
fprintf('%.4f  ', abs(eigs_cl));
fprintf('\n  max |eig| = %.4f  (must be < 1)\n\n', max(abs(eigs_cl)));

%% ---------------- 9. SAVE ----------------------------------------------
save('static_kf.mat','A','B','H','Q','R','P0','P_inf','K_inf','Ts');
fprintf('Saved static_kf.mat\n');