#!/usr/bin/env python3
"""Synthetic checks for src/scripts/analyze_calibration.py.

Run with the repository Python (numpy is required):

    python3 tests/calibration_host/test_analysis.py

The tests build calibration rows directly, so no robot or serial port is
needed. They verify the axis search, the noise estimates and the step metrics
against known inputs.
"""

import contextlib
import importlib.util
import io
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
ANALYZER = ROOT / "src" / "scripts" / "analyze_calibration.py"

GEAR = 9.6
WHEEL_RADIUS = 0.0315
WHEEL_BASE = 0.14
N_COLUMNS = 38


def load_analyzer():
    spec = importlib.util.spec_from_file_location("analyze_calibration", ANALYZER)
    module = importlib.util.module_from_spec(spec)
    sys.dont_write_bytecode = True
    spec.loader.exec_module(module)
    return module


def quiet(fn, *args, **kwargs):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*args, **kwargs)


def blank(n):
    return np.zeros((n, N_COLUMNS))


def run_mag_checks(module):
    n = 2000
    t = np.linspace(0.0, 10.0, n)
    theta = 2.0 * np.pi * np.clip((t - 1.0) / 8.0, 0.0, 1.0)
    yaw_rate = np.gradient(theta, t)

    data = blank(n)
    data[:, 0] = t
    data[:, 1] = 1
    data[:, 2] = 0x9F  # accel/gyro/mag fresh+valid+available, AS5 valid
    data[:, 12] = -np.rad2deg(yaw_rate) + 0.05  # gyro X, deg/s
    data[:, 13] = 0.0
    data[:, 14] = 0.0
    data[:, 15] = -1.0  # X is down/vertical
    data[:, 16] = 0.0
    data[:, 17] = 0.0
    data[:, 18] = 20.0
    data[:, 19] = 30.0 * np.sin(theta)
    data[:, 20] = -30.0 * np.cos(theta)
    data[:, 21:23] = 0.0
    data[:, 23:25] = 0.0
    data[:, 25] = 0.010
    data[:, 26] = np.arctan2(data[:, 19], -data[:, 20])
    data[:, 26] += np.random.default_rng(3).normal(0.0, 0.005, n)
    data[:, 34:38] = 1.0

    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "cal.log"
        with open(path, "w") as f:
            f.write("% CAL mag\n")
            f.write("% synthetic\n")
            np.savetxt(f, data, fmt="%.9g", delimiter=" ")
        loaded, mode = module.load_cal(path)
        assert mode == "mag"
        assert loaded.shape == (n, N_COLUMNS)

    result = quiet(module.analyze_mag, t, data)
    assert result["gravity_axis"] == 0, result["gravity_axis"]
    assert result["vertical_axis"] == 0, result["vertical_axis"]
    assert abs(result["firmware_slope"] - 1.0) < 0.1, result["firmware_slope"]
    assert result["firmware_ok"], result
    assert result["best"]["axes"] == (1, 2), result["best"]
    assert abs(abs(result["best"]["slope"]) - 1.0) < 0.1, result["best"]


def run_noise_checks(module):
    n = 3000
    rng = np.random.default_rng(5)
    data = blank(n)
    data[:, 0] = np.arange(n) * 0.001
    data[:, 1] = 2
    flags = np.full(n, 0x01 | 0x02 | 0x20, dtype=np.int64)  # accel, gyro, AS5 valid
    flags[::10] |= 0x04 | 0x08 | 0x10  # mag fresh/valid/available
    flags[::10] |= 0x40               # AS5 velocity fresh
    data[:, 2] = flags

    data[:, 12] = rng.normal(0.0, 0.1, n)          # gyro X, deg/s
    data[:, 15] = rng.normal(0.0, 0.02, n)
    data[:, 16] = rng.normal(0.0, 0.02, n)
    data[:, 17] = rng.normal(0.0, 0.0285 / 9.80665, n)

    sigma_angle = np.deg2rad(0.068)
    data[:, 21] = rng.normal(0.0, sigma_angle, n)
    data[:, 22] = rng.normal(0.0, sigma_angle, n)
    sigma_v = np.sqrt(2.0) * WHEEL_RADIUS * sigma_angle / (GEAR * 0.010)
    data[:, 23] = rng.normal(0.0, sigma_v, n)
    data[:, 24] = rng.normal(0.0, sigma_v, n)
    data[:, 25] = 0.010
    data[:, 26] = rng.normal(0.0, 0.01, n)

    kf_mask = np.full(n, 0x03, dtype=np.int64)  # gyro + wheel v
    kf_mask[::10] |= 0x04  # wheel omega
    kf_mask[::10] |= 0x08  # mag
    data[:, 3] = kf_mask
    data[:, 30] = rng.normal(0.0, 1.0, n)  # gyro innovation
    data[:, 31] = rng.normal(0.0, 1.0, n)  # wheel v innovation
    data[:, 32] = rng.normal(0.0, 1.0, n)  # wheel omega innovation
    data[:, 33] = rng.normal(0.0, 1.0, n)  # mag innovation
    data[:, 34:38] = 1.0

    result = quiet(module.analyze_noise, data[:, 0], data)
    checks = result["checks"]
    for name in ("gyro yaw (deg/s)", "accel forward (m/s^2)",
                 "AS5X47 angle (rad)", "wheel v (m/s)",
                 "wheel omega (rad/s)", "mag heading (rad)"):
        ratio = checks[name]["ratio"]
        assert 0.5 <= ratio <= 2.0, (name, ratio)
    for row in result["nis"]:
        if row["label"] != "mag rejected":
            assert 0.7 <= row["mean"] <= 1.3, row


def run_track_checks(module):
    n = 3000
    t = np.arange(n) / 500.0
    speeds = [0.0, 0.05, 0.10, 0.15, 0.0, -0.05, -0.10, -0.15, 0.0]
    durations = [0.3, 0.8, 0.8, 0.8, 0.5, 0.8, 0.8, 0.8, 0.4]
    sp = np.zeros(n)
    edge = 0.0
    for speed, duration in zip(speeds, durations):
        i0 = int(round(edge * 500.0))
        edge += duration
        i1 = min(n, int(round(edge * 500.0)))
        sp[i0:i1] = speed * GEAR / WHEEL_RADIUS
    # First-order motor response, tau = 80 ms.
    y = np.zeros(n)
    dt = 1.0 / 500.0
    tau = 0.08
    for k in range(1, n):
        y[k] = y[k - 1] + dt / tau * (sp[k] - y[k - 1])

    data = blank(n)
    data[:, 0] = t
    data[:, 1] = 3
    data[:, 2] = 0x9F
    data[:, 4] = sp
    data[:, 5] = sp
    data[:, 6] = y
    data[:, 7] = y
    data[:, 8] = np.clip(0.02 * (sp - y), -3.0, 3.0)
    data[:, 9] = np.clip(0.02 * (sp - y), -3.0, 3.0)
    data[:, 10] = y
    data[:, 11] = y
    data[:, 25] = 0.010
    data[:, 23] = y * WHEEL_RADIUS / GEAR
    data[:, 24] = y * WHEEL_RADIUS / GEAR
    data[:, 27] = y * WHEEL_RADIUS / GEAR  # KF v
    data[:, 34:38] = 1.0

    result = quiet(module.analyze_track, t, data)
    steps = [m for m in result["steps"] if m["signal"] == "feedback"]
    assert len(steps) >= 6, len(steps)
    summary = result["summary"]["feedback"]
    assert 0.10 <= summary["median_rise"] <= 0.30, summary
    assert summary["median_abs_ss_error"] < 0.5, summary
    assert summary["mean_saturation"] == 0.0, summary
    # The independent AS5X47 and KF signals are identical in this fixture.
    assert result["summary"]["as5x47"]["n"] == len(steps)


def main():
    module = load_analyzer()
    run_mag_checks(module)
    run_noise_checks(module)
    run_track_checks(module)
    print("PASS: calibration analyzer synthetic checks")
    return 0


if __name__ == "__main__":
    sys.exit(main())
