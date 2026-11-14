#!/usr/bin/env python3
"""Synthetic checks for src/scripts/analyze_stream.py.

Run with the repository Python (numpy is required):

    python3 tests/calibration_host/test_stream.py

The fixture is a 10-minute stationary stream with known white noise on the
yaw gyro and the forward accelerometer, so the recovered 1-sigma values and
the Allan noise density have known answers.
"""

import contextlib
import importlib.util
import io
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
ANALYZER = ROOT / "src" / "scripts" / "analyze_stream.py"
N_COLUMNS = 19


def load_analyzer():
    spec = importlib.util.spec_from_file_location("analyze_stream", ANALYZER)
    module = importlib.util.module_from_spec(spec)
    sys.dont_write_bytecode = True
    spec.loader.exec_module(module)
    return module


def quiet(fn, *args, **kwargs):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*args, **kwargs)


def make_stream(n=60000, dt=0.01, seed=7):
    rng = np.random.default_rng(seed)
    t = np.arange(n) * dt
    data = np.zeros((n, N_COLUMNS))
    data[:, 0] = t
    data[:, 1] = dt * 1e6
    data[:, 2] = 0x01 | 0x02  # accel and gyro fresh; no magnetometer
    data[:, 5] = rng.normal(0.0, 0.1, n)             # gyro x, deg/s
    data[:, 6] = rng.normal(0.0, 0.2, n)
    data[:, 7] = rng.normal(0.0, 0.1, n)
    data[:, 8] = rng.normal(0.0, 0.02, n)            # accel x, g
    data[:, 9] = rng.normal(0.0, 0.02, n)
    data[:, 10] = rng.normal(0.0, 0.03 / 9.80665, n)  # accel z (forward), g
    data[:, 18] = 25.0
    return t, data


def run_stream_checks(module):
    t, data = make_stream()
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "stream.log"
        with open(path, "w") as f:
            f.write("% CAL stream 100 Hz (divider 10), cap 1200 s\n")
            f.write("% 1 time (s)\n")
            f.write("% 2 IMU sample interval (us)\n")
            f.write("% 3 flags (same bits as the calibration capture)\n")
            f.write("% 4 AK8963 ST1\n")
            f.write("% 5 AK8963 ST2\n")
            np.savetxt(f, data, fmt="%.9g", delimiter=" ")
        loaded = module.load_stream(path)
    assert loaded.shape == (len(t), N_COLUMNS), loaded.shape

    result = quiet(module.analyze_stream, t, loaded)
    assert abs(result["rate"] - 100.0) < 0.1, result["rate"]
    assert abs(result["duration"] - 599.99) < 0.02, result["duration"]

    gyro_sigma = result["sigma"]["gyro_yaw_deg"]
    accel_sigma = result["sigma"]["accel_forward"]
    assert abs(gyro_sigma - 0.1) < 0.01, gyro_sigma
    assert abs(accel_sigma - 0.03) < 0.003, accel_sigma

    assert result["mag_valid"] == 0, result["mag_valid"]
    assert result["mag_attempts"] == 0, result["mag_attempts"]
    assert result["allan"]["gyro yaw"]["tau"].size > 5

    # White rate noise: adev = q/sqrt(tau) with q = sigma * sqrt(dt).
    gyro_allan = result["allan"]["gyro yaw"]
    expected_q = 0.1 * np.pi / 180.0 * np.sqrt(0.01)
    assert abs(gyro_allan["slope"] + 0.5) < 0.1, gyro_allan["slope"]
    assert abs(gyro_allan["q"] / expected_q - 1.0) < 0.15, gyro_allan["q"]

    accel_allan = result["allan"]["accel forward"]
    expected_qa = 0.03 * np.sqrt(0.01)
    assert abs(accel_allan["slope"] + 0.5) < 0.1, accel_allan["slope"]
    assert abs(accel_allan["q"] / expected_qa - 1.0) < 0.15, accel_allan["q"]


def main():
    module = load_analyzer()
    run_stream_checks(module)
    print("PASS: stream analyzer synthetic checks")
    return 0


if __name__ == "__main__":
    sys.exit(main())
