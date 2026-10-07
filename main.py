import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# 1. Electrical Parameters
# Resistance R = V / I
V15, A15 = 1.5, 0.15
V3, A3 = 3.0, 0.32
V45, A45 = 4.5, 0.67

R15 = V15 / A15 
R3 = V3 / A3 
R45 = V45 / A45

# Using the specific R values from the original script
R_left = 6.5217
R_right = 2.8846

# Inductance L = tau_e * R (where tau_e is the electrical time constant)
tau_e = 0.0225
L_left = tau_e * R_left
L_right = tau_e * R_right

# 2. Data Loading and Masking
repo_root = next(path for path in (Path.cwd(), *Path.cwd().parents)
                 if (path / "src" / "logs").is_dir())
log_path = repo_root / "src" / "logs" / "device-monitor-260909-113357.log"
data = np.loadtxt(log_path, comments="%")

time_s = data[:, 0]
voltage_v = data[:, 3:5]
velocity_rad_s = data[:, 7:9]
current_a = data[:, 14:16]

step_start_s = 0.020
step_end_s = 0.500
window_s = 0.012

baseline_mask = (time_s >= step_start_s - window_s) & (time_s < step_start_s)
steady_mask = (time_s >= step_end_s - window_s) & (time_s <= step_end_s)

baseline_current_a = np.mean(current_a[baseline_mask, :], axis=0)

voltage_ss = np.mean(voltage_v[steady_mask, :], axis=0)
omega_ss = np.mean(velocity_rad_s[steady_mask, :], axis=0)
current_ss_raw = np.mean(current_a[steady_mask, :], axis=0)

current_ss_true = current_ss_raw - baseline_current_a

# 3. Motor Parameters (Kb, Kt, D)
# Note: Kt is assumed equal to Kb in standard SI units
V_back_emf_left = voltage_ss[0] - (current_ss_true[0] * R_left)
Kb_left = V_back_emf_left / omega_ss[0]
Kt_left = Kb_left 
D_left = (current_ss_true[0] * Kt_left) / omega_ss[0]

V_back_emf_right = voltage_ss[1] - (current_ss_true[1] * R_right)
Kb_right = V_back_emf_right / omega_ss[1]
Kt_right = Kb_right
D_right = (current_ss_true[1] * Kt_right) / omega_ss[1]

print(f"Left motor: Kb = {Kb_left:.4e}, D = {D_left:.4e}")
print(f"Right motor: Kb = {Kb_right:.4e}, D = {D_right:.4e}")

# 4. Solving for Inertia (J)
def calculate_motor_inertia(R, L, K_t, K_b, D, s):
    """
    Calculates the motor inertia (J) based on the system's characteristic equation.
    """
    numerator = -(D * L * s + D * R + K_t * K_b)
    denominator = s * (L * s + R)
    return numerator / denominator

# The dominant pole 's' is determined from the mechanical time constant (tau_m).
# For demonstration, a placeholder value is used. This must be measured from the velocity step response.
tau_m_left = 0.088 
s_left = -1.0 / tau_m_left

J_left = calculate_motor_inertia(R=R_left, L=L_left, K_t=Kt_left, K_b=Kb_left, D=D_left, s=s_left)
print(f"Left motor: J = {J_left:.4e}")

# 5. Verify system poles using numpy.roots
# Characteristic polynomial: s^2*(J*L) + s*(D*L + J*R) + (D*R + Kt*Kb) = 0
poly_left = [
    J_left * L_left, 
    D_left * L_left + J_left * R_left, 
    D_left * R_left + Kt_left * Kb_left
]
roots_left = np.roots(poly_left)
print(f"Left motor system poles: {roots_left}")
