"""NumPy reference implementation of the 7-state differential-drive filter.

State: x = [s, theta, v, omega, b_a, b_g, b_m]

Sensors: MPU-9250 accelerometer / gyroscope / magnetometer and two AS5X47
absolute angle sensors.  The AS5X47 angles are differenced over a 10 ms window
to give instantaneous forward velocity and yaw rate; no wheel-derived distance
or heading is used.

This file is both a library (``DiffDriveKF``) and a headless demo.  Constants
and update order mirror ``src/src/kalman.{h,cpp}`` so ``tests/kalman_host/
test_parity.py`` can compare the two implementations step by step.
"""

import math

import numpy as np

# ----------------------------------------------------------------------
# Datasheet-derived noise parameters
# ----------------------------------------------------------------------
# MPU-9250 accel noise density 300 ug/sqrt(Hz), DLPF 94 Hz.
SIGMA_ACCEL = 300e-6 * 9.80665 * math.sqrt(94.0)   # m/s^2
# MPU-9250 gyro total RMS noise 0.1 deg/s, DLPF 92 Hz.
SIGMA_GYRO = 0.1 * math.pi / 180.0                 # rad/s
# AS5X47 RMS output angle noise 0.068 deg (1 sigma, NOISESET=0).
SIGMA_ANGLE = 0.068 * math.pi / 180.0              # rad
# Magnetometer heading noise: no datasheet figure; conservative estimate.
SIGMA_MAG = 0.01                                   # rad

WHEEL_RADIUS = 0.0315                              # m
WHEEL_BASE = 0.14                                  # m
WHEEL_GEAR = 9.6                                   # motor rev / wheel rev
WHEEL_WINDOW = 0.01                                # s (100 Hz differencing)

# Measurement variances.
R_GYRO = SIGMA_GYRO ** 2                           # (rad/s)^2
R_MAG = SIGMA_MAG ** 2                             # rad^2

# Process-noise random walks (tuning values, not datasheet numbers).
Q_V = 1e-2        # (m/s)^2 / s
Q_OMEGA = 1e-2    # (rad/s)^2 / s
Q_BA = 1e-6       # (m/s^2)^2 / s
Q_BG = 1e-8       # (rad/s)^2 / s
Q_BM = 1e-10      # rad^2 / s

P0_DIAG = np.array([1e-4, 1e-2, 1e-2, 1e-2, 4e-2, 1e-4, 1e-2])

N = 7
S, THETA, V, OMEGA, BA, BG, BM = range(N)

TWO_PI = 2.0 * math.pi
PI = math.pi


def wrap(a):
    """Fold an angle into (-pi, pi], exactly like the C ``fmodf`` version."""
    a = math.fmod(float(a), TWO_PI)
    if a <= -PI:
        a += TWO_PI
    elif a > PI:
        a -= TWO_PI
    return a


def wrap_array(a):
    """Vectorised :func:`wrap` for NumPy arrays."""
    a = np.fmod(np.asarray(a, dtype=float), TWO_PI)
    a[a <= -PI] += TWO_PI
    a[a > PI] -= TWO_PI
    return a


class DiffDriveKF:
    """7-state Kalman filter, one scalar measurement update at a time."""

    def __init__(self):
        self.x = np.zeros(N)
        self.P = np.zeros((N, N))
        self.mag_rejected = False
        self.healthy = True
        self.reset()

    # -- lifecycle -----------------------------------------------------
    def reset(self, heading0=0.0):
        self.x = np.zeros(N)
        self.x[THETA] = float(heading0)
        self.P = np.diag(P0_DIAG.copy())
        self.mag_rejected = False
        self.healthy = True

    # -- accessors -----------------------------------------------------
    @property
    def state(self):
        return self.x.copy()

    @property
    def covariance(self):
        return self.P.copy()

    # -- prediction ----------------------------------------------------
    def predict(self, a_meas, dt):
        """Propagate with measured forward acceleration.  False if rejected."""
        a = float(a_meas)
        dt = float(dt)
        if not (math.isfinite(a) and math.isfinite(dt)):
            return False
        if dt <= 0.0 or dt > 0.05:
            return False

        F = np.eye(N)
        F[S, V] = dt
        F[THETA, OMEGA] = dt
        F[V, BA] = -dt

        B = np.zeros(N)
        B[V] = dt

        self.x = F @ self.x + B * a

        sigma_a2 = SIGMA_ACCEL ** 2
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2

        Q = np.zeros((N, N))
        Q[S, S] = 0.25 * sigma_a2 * dt4
        Q[S, V] = Q[V, S] = 0.5 * sigma_a2 * dt3
        Q[V, V] = sigma_a2 * dt2 + Q_V * dt
        Q[OMEGA, OMEGA] = Q_OMEGA * dt
        Q[BA, BA] = Q_BA * dt
        Q[BG, BG] = Q_BG * dt
        Q[BM, BM] = Q_BM * dt

        self.P = F @ self.P @ F.T + Q
        self._symmetrize()
        if not np.all(np.isfinite(self.x)):
            self.healthy = False
        return True

    # -- measurements --------------------------------------------------
    def update_gyro(self, z):
        H = np.zeros(N)
        H[OMEGA] = 1.0
        H[BG] = 1.0
        self._update_scalar(H, z, R_GYRO)

    def update_wheel_v(self, z, window=WHEEL_WINDOW):
        H = np.zeros(N)
        H[V] = 1.0
        r = WHEEL_RADIUS * SIGMA_ANGLE / (WHEEL_GEAR * float(window))
        self._update_scalar(H, z, r * r)

    def update_wheel_omega(self, z, window=WHEEL_WINDOW):
        H = np.zeros(N)
        H[OMEGA] = 1.0
        r = 2.0 * WHEEL_RADIUS * SIGMA_ANGLE / (
            WHEEL_BASE * WHEEL_GEAR * float(window))
        self._update_scalar(H, z, r * r)

    def update_mag(self, z):
        z = float(z)
        H = np.zeros(N)
        H[THETA] = 1.0
        H[BM] = 1.0

        Hx = float(H @ self.x)
        y = wrap(z - Hx)

        PHt = self.P @ H
        S = R_MAG + float(H @ PHt)

        if y * y > 9.0 * S:
            self.mag_rejected = True
            return

        self.mag_rejected = False
        self._update_scalar(H, Hx + wrap(z - Hx), R_MAG)

    # -- internals -----------------------------------------------------
    def _update_scalar(self, H, z, R):
        z = float(z)
        PHt = self.P @ H
        S = float(R) + float(H @ PHt)

        if not math.isfinite(S) or S <= 0.0 or not math.isfinite(z):
            return

        innovation = z - float(H @ self.x)
        K = PHt / S

        self.x = self.x + K * innovation
        self.P = self.P - np.outer(K, H @ self.P)
        self._symmetrize()

        if not np.all(np.isfinite(self.x)):
            self.healthy = False

    def _symmetrize(self):
        self.P = 0.5 * (self.P + self.P.T)


# ----------------------------------------------------------------------
# Demonstration: MPU-9250 + 2x AS5X47, 1 kHz
# ----------------------------------------------------------------------
DT = 0.001
N_DEMO = 20000            # 20 s at 1 kHz


def simulate(n=N_DEMO, dt=DT, seed=42):
    """Build the true trajectory and the simulated sensor streams."""
    rng = np.random.default_rng(seed)
    t = np.arange(n) * dt

    # Constant forward velocity and yaw rate; heading crosses +pi once.
    v_true = np.full(n, 0.30)
    omega_true = np.full(n, 0.35)
    s_true = np.cumsum(v_true) * dt
    theta_true = np.cumsum(omega_true) * dt

    b_a_true = 0.05       # m/s^2
    b_g_true = 0.01       # rad/s
    b_m_true = 0.10       # rad

    a_true = np.zeros(n)  # constant velocity

    # MPU-9250 streams.
    a_meas = a_true + b_a_true + rng.normal(0.0, SIGMA_ACCEL, n)
    gyro_meas = omega_true + b_g_true + rng.normal(0.0, SIGMA_GYRO, n)
    mag_meas = wrap_array(theta_true + b_m_true + rng.normal(0.0, SIGMA_MAG, n))

    # AS5X47 streams: wheel angles (single turn) from wheel speed.
    v_left = v_true - 0.5 * WHEEL_BASE * omega_true
    v_right = v_true + 0.5 * WHEEL_BASE * omega_true
    theta_left = np.cumsum(v_left / WHEEL_RADIUS * WHEEL_GEAR) * dt
    theta_right = np.cumsum(v_right / WHEEL_RADIUS * WHEEL_GEAR) * dt
    left_meas = wrap_array(theta_left + rng.normal(0.0, SIGMA_ANGLE, n))
    right_meas = wrap_array(theta_right + rng.normal(0.0, SIGMA_ANGLE, n))

    return {
        "t": t,
        "v_true": v_true,
        "omega_true": omega_true,
        "s_true": s_true,
        "theta_true": theta_true,
        "b_a_true": b_a_true,
        "b_g_true": b_g_true,
        "b_m_true": b_m_true,
        "a_meas": a_meas,
        "gyro_meas": gyro_meas,
        "mag_meas": mag_meas,
        "left_meas": left_meas,
        "right_meas": right_meas,
    }


def run_demo(sim):
    """Run the filter over the simulated streams, returning estimates."""
    n = len(sim["t"])
    kf = DiffDriveKF()
    kf.reset(0.0)

    window_steps = int(round(WHEEL_WINDOW / DT))
    window = window_steps * DT

    est = np.zeros((n, N))
    rejected = np.zeros(n, dtype=bool)

    for i in range(n):
        kf.predict(sim["a_meas"][i], DT)
        kf.update_gyro(sim["gyro_meas"][i])

        if i % 10 == 0 and i >= window_steps:
            dl = wrap(float(sim["left_meas"][i]) -
                      float(sim["left_meas"][i - window_steps]))
            dr = wrap(float(sim["right_meas"][i]) -
                      float(sim["right_meas"][i - window_steps]))
            v_meas = WHEEL_RADIUS * (dl + dr) / (2.0 * WHEEL_GEAR * window)
            w_meas = WHEEL_RADIUS * (dr - dl) / (
                WHEEL_BASE * WHEEL_GEAR * window)
            kf.update_wheel_v(v_meas, window)
            kf.update_wheel_omega(w_meas, window)

        if i % 100 == 0:
            kf.update_mag(sim["mag_meas"][i])

        est[i] = kf.state
        rejected[i] = kf.mag_rejected

    return est, rejected


def _plot(sim, est, rejected):
    """Save a PNG if matplotlib is available; never raise on a headless host."""
    import os

    try:
        import matplotlib
        matplotlib.use(os.environ.get("MPLBACKEND", "Agg"))
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; skipping plot")
        return None

    t = sim["t"]
    fig, axs = plt.subplots(3, 2, figsize=(12, 8))

    axs[0, 0].plot(t, sim["s_true"], "k", label="true")
    axs[0, 0].plot(t, est[:, S], "r--", label="est")
    axs[0, 0].set_ylabel("s [m]")
    axs[0, 0].set_title("Distance")
    axs[0, 0].legend()

    axs[0, 1].plot(t, sim["theta_true"], "k", label="true")
    axs[0, 1].plot(t, est[:, THETA], "r--", label="est")
    axs[0, 1].axhline(PI, color="0.7", lw=0.8)
    axs[0, 1].axhline(-PI, color="0.7", lw=0.8)
    axs[0, 1].set_ylabel("theta [rad]")
    axs[0, 1].set_title("Heading (wrap crossing)")
    axs[0, 1].legend()

    axs[1, 0].plot(t, sim["v_true"], "k", label="true")
    axs[1, 0].plot(t, est[:, V], "r--", label="est")
    axs[1, 0].set_ylabel("v [m/s]")
    axs[1, 0].set_title("Forward velocity")
    axs[1, 0].legend()

    axs[1, 1].plot(t, sim["omega_true"], "k", label="true")
    axs[1, 1].plot(t, est[:, OMEGA], "r--", label="est")
    axs[1, 1].set_ylabel("omega [rad/s]")
    axs[1, 1].set_title("Turn rate")
    axs[1, 1].legend()

    axs[2, 0].plot(t, np.full(len(t), sim["b_a_true"]), "k", label="true")
    axs[2, 0].plot(t, est[:, BA], "r--", label="est")
    axs[2, 0].set_ylabel("b_a [m/s^2]")
    axs[2, 0].set_xlabel("t [s]")
    axs[2, 0].set_title("Accel bias")
    axs[2, 0].legend()

    axs[2, 1].plot(t, np.full(len(t), sim["b_g_true"]), "k", label="b_g true")
    axs[2, 1].plot(t, est[:, BG], "r--", label="b_g est")
    axs[2, 1].plot(t, np.full(len(t), sim["b_m_true"]), "b", label="b_m true")
    axs[2, 1].plot(t, est[:, BM], "c--", label="b_m est")
    axs[2, 1].set_ylabel("bias [rad/s or rad]")
    axs[2, 1].set_xlabel("t [s]")
    axs[2, 1].set_title("Gyro and mag biases")
    axs[2, 1].legend()

    fig.tight_layout()
    path = os.path.join("/tmp", "kalman_reference_demo.png")
    try:
        fig.savefig(path, dpi=110)
    except OSError:
        print("could not save plot")
        return None
    finally:
        plt.close(fig)
    return path


def main():
    sim = simulate()
    est, rejected = run_demo(sim)

    settle = int(5.0 / DT)
    heading_err = np.abs(wrap_array(est[settle:, THETA] -
                                    sim["theta_true"][settle:]))
    v_err = np.abs(est[settle:, V] - sim["v_true"][settle:])

    print(f"{len(sim['t'])} steps at {1.0 / DT:.0f} Hz")
    print(f"final s      : true {sim['s_true'][-1]:.4f} m   "
          f"est {est[-1, S]:.4f} m")
    print(f"final v      : true {sim['v_true'][-1]:.4f} m/s  "
          f"est {est[-1, V]:.4f} m/s")
    print(f"max |v err| after 5 s        : {v_err.max():.2e} m/s")
    print(f"max |wrap(heading err)| after 5 s: {heading_err.max():.2e} rad")
    print(f"estimated biases: b_a={est[-1, BA]:.4f} m/s^2  "
          f"b_g={est[-1, BG]:.4f} rad/s  b_m={est[-1, BM]:.4f} rad")
    print("note: the mag measures theta + b_m, so a constant heading offset "
          "and b_m are not separable; the split follows the prior.")
    print(f"mag rejections: {int(rejected.sum())}")

    path = _plot(sim, est, rejected)
    if path:
        print(f"plot saved to {path}")


if __name__ == "__main__":
    main()
