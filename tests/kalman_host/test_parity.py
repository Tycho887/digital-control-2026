#!/usr/bin/env python3
"""Step-by-step parity check: C++ stationary estimator vs. NumPy reference.

Run with the host venv Python:

    tests/kalman_host/.venv/bin/python tests/kalman_host/test_parity.py

The script builds ``tests/kalman_host/parity.cpp`` together with
``src/src/kalman.cpp`` and runs it on a deterministic 3000-step scenario.
"""

import csv
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from verify_gains import load_gains, wrap  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
N_STEPS = 3000
DT = 0.001
WHEEL_WINDOW = 0.01
MAG_OUTLIER_STEP = 1500
SEED = 7

SIGMA_ACCEL = 300e-6 * 9.80665 * np.sqrt(94.0)
SIGMA_GYRO = 0.1 * np.pi / 180.0
SIGMA_MAG = 0.01

STATE_NAMES = ["v", "b_a", "theta", "omega", "b_g"]
INNOV_NAMES = ["innov_gyro", "innov_wheel_v", "innov_wheel_omega", "innov_mag"]


class StaticKF:
    """NumPy reference matching src/src/kalman.cpp."""

    def __init__(self, gains):
        self.g = gains
        self.v = 0.0
        self.b_a = 0.0
        self.theta = 0.0
        self.omega = 0.0
        self.b_g = 0.0
        self.innov = {name: 0.0 for name in INNOV_NAMES}
        self.healthy = True

    def predict(self, a, dt):
        if not (np.isfinite(a) and np.isfinite(dt)) or dt <= 0.0 or dt > 0.05:
            return False
        self.v += (a - self.b_a) * dt
        self.theta = wrap(self.theta + self.omega * dt)
        return self.healthy

    def update_gyro(self, z):
        innov = z - (self.omega + self.b_g)
        self.theta = wrap(self.theta + self.g["K_GYRO"][0] * innov)
        self.omega += self.g["K_GYRO"][1] * innov
        self.b_g += self.g["K_GYRO"][2] * innov
        self.innov["innov_gyro"] = innov
        return True

    def update_wheel_v(self, z):
        innov = z - self.v
        self.v += self.g["K_WHEEL_V"][0] * innov
        self.b_a += self.g["K_WHEEL_V"][1] * innov
        self.innov["innov_wheel_v"] = innov
        return True

    def update_wheel_omega(self, z):
        innov = z - self.omega
        self.theta = wrap(self.theta + self.g["K_WHEEL_OMEGA"][0] * innov)
        self.omega += self.g["K_WHEEL_OMEGA"][1] * innov
        self.b_g += self.g["K_WHEEL_OMEGA"][2] * innov
        self.innov["innov_wheel_omega"] = innov
        return True

    def update_mag(self, z):
        innov = wrap(z - self.theta)
        self.innov["innov_mag"] = innov
        if abs(innov) > 0.25:
            return False
        self.theta = wrap(self.theta + self.g["K_MAG"][0] * innov)
        self.omega += self.g["K_MAG"][1] * innov
        self.b_g += self.g["K_MAG"][2] * innov
        return True


def build_scenario(path):
    rng = np.random.default_rng(SEED)
    t = np.arange(N_STEPS) * DT
    v = 0.2 + 0.05 * np.sin(2.0 * np.pi * 0.2 * t)
    omega = 0.1 * np.sin(2.0 * np.pi * 0.05 * t)
    theta = np.cumsum(omega * DT)

    a_true = np.zeros(N_STEPS)
    a_true[1:] = np.diff(v) / DT

    b_a_true = 0.05
    b_g_true = 0.01

    a_meas = a_true + b_a_true + rng.normal(0.0, SIGMA_ACCEL, N_STEPS)
    gyro = omega + b_g_true + rng.normal(0.0, SIGMA_GYRO, N_STEPS)
    mag = np.array([wrap(x) for x in
                    theta + rng.normal(0.0, SIGMA_MAG, N_STEPS)])

    sigma_wheel_v = (np.sqrt(2.0) * 0.0315 * (0.068 * np.pi / 180.0) /
                     (9.6 * WHEEL_WINDOW))
    sigma_wheel_omega = (np.sqrt(2.0) * 2.0 * 0.0315 *
                         (0.068 * np.pi / 180.0) /
                         (0.14 * 9.6 * WHEEL_WINDOW))
    wheel_v = v + rng.normal(0.0, sigma_wheel_v, N_STEPS)
    wheel_omega = omega + rng.normal(0.0, sigma_wheel_omega, N_STEPS)

    use_gyro = np.ones(N_STEPS, dtype=int)
    use_wheel = (np.arange(N_STEPS) % 10 == 0).astype(int)
    use_mag = (np.arange(N_STEPS) % 100 == 0).astype(int)

    if MAG_OUTLIER_STEP < N_STEPS:
        mag[MAG_OUTLIER_STEP] = wrap(mag[MAG_OUTLIER_STEP] + np.pi)

    with open(path, "w", newline="") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(["step", "dt", "a", "gyro", "wheel_v", "wheel_omega",
                         "mag", "use_gyro", "use_wheel", "use_mag"])
        for i in range(N_STEPS):
            writer.writerow([i, f"{DT:.9g}", f"{a_meas[i]:.9g}",
                             f"{gyro[i]:.9g}", f"{wheel_v[i]:.9g}",
                             f"{wheel_omega[i]:.9g}", f"{mag[i]:.9g}",
                             int(use_gyro[i]), int(use_wheel[i]),
                             int(use_mag[i])])


def read_scenario(path):
    rows = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            rows.append({
                "dt": float(row["dt"]),
                "a": float(row["a"]),
                "gyro": float(row["gyro"]),
                "wheel_v": float(row["wheel_v"]),
                "wheel_omega": float(row["wheel_omega"]),
                "mag": float(row["mag"]),
                "use_gyro": int(row["use_gyro"]),
                "use_wheel": int(row["use_wheel"]),
                "use_mag": int(row["use_mag"]),
            })
    return rows


def replay(rows, gains):
    kf = StaticKF(gains)
    out = []
    for row in rows:
        kf.predict(row["a"], row["dt"])
        if row["use_gyro"]:
            kf.update_gyro(row["gyro"])
        if row["use_wheel"]:
            kf.update_wheel_v(row["wheel_v"])
            kf.update_wheel_omega(row["wheel_omega"])
        if row["use_mag"]:
            kf.update_mag(row["mag"])
        out.append([kf.v, kf.b_a, kf.theta, kf.omega, kf.b_g,
                    kf.innov["innov_gyro"], kf.innov["innov_wheel_v"],
                    kf.innov["innov_wheel_omega"], kf.innov["innov_mag"],
                    int(kf.healthy)])
    return np.array(out)


def read_cpp(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    cols = STATE_NAMES + INNOV_NAMES + ["healthy"]
    return np.array([[float(row[c]) for c in cols] for row in rows])


def main():
    gains = load_gains()
    exe = Path(tempfile.gettempdir()) / "kalman-parity-static"
    build = subprocess.run(
        ["c++", "-std=c++14", "-O2", f"-I{ROOT / 'src' / 'src'}",
         str(ROOT / "tests" / "kalman_host" / "parity.cpp"),
         str(ROOT / "src" / "src" / "kalman.cpp"), "-o", str(exe)],
        capture_output=True, text=True)
    if build.returncode != 0:
        print(build.stderr)
        return 1

    with tempfile.TemporaryDirectory() as tmp:
        scenario = Path(tmp) / "scenario.csv"
        output = Path(tmp) / "output.csv"
        build_scenario(scenario)
        run = subprocess.run([str(exe), str(scenario), str(output)],
                             capture_output=True, text=True)
        if run.returncode != 0:
            print(run.stderr)
            return 1
        cpp = read_cpp(output)
        ref = replay(read_scenario(scenario), gains)

    if cpp.shape != ref.shape:
        print(f"FAIL: shape mismatch {cpp.shape} vs {ref.shape}")
        return 1

    labels = STATE_NAMES + INNOV_NAMES
    for i, label in enumerate(labels):
        if not np.allclose(cpp[:, i], ref[:, i], rtol=1e-4, atol=2e-5):
            diff = np.max(np.abs(cpp[:, i] - ref[:, i]))
            print(f"FAIL: {label} mismatch, max |diff| = {diff:.3g}")
            return 1

    if not np.all(cpp[:, -1] == ref[:, -1]):
        print("FAIL: healthy flag mismatch")
        return 1

    print(f"PASS: {N_STEPS} steps, C++ and NumPy agree on states and innovations.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
