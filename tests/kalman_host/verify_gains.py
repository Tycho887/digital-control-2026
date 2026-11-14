#!/usr/bin/env python3
"""Verify the stationary gain bank at the real multi-rate update schedule.

The firmware predicts at 1 kHz, applies the gyro update every step, the wheel
v/omega updates every 10 ms, and the magnetometer update asynchronously. A
single DARE solved at one rate cannot be applied unchanged at all three rates,
so this script checks the closed-loop error transition over a full 100-step
cycle instead.

Run with the host venv Python (numpy only):

    tests/kalman_host/.venv/bin/python tests/kalman_host/verify_gains.py
"""

import math
import re
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
HEADER = ROOT / "src" / "src" / "kalman.h"

DT = 0.001
WHEEL_DIVIDER = 10
MAG_DIVIDER = 100

# State order used by the checks: [v, b_a, theta, omega, b_g].
V, BA, THETA, OMEGA, BG = range(5)


def parse_gain(header, name, count):
    match = re.search(
        rf"{name}\s*\[\s*{count}\s*\]\s*=\s*\{{([^}}]*)\}}", header)
    if match is None:
        raise RuntimeError(f"could not find {name}[{count}] in {HEADER}")
    values = [float(x.strip().rstrip("fF")) for x in match.group(1).split(",")]
    if len(values) != count:
        raise RuntimeError(f"{name}: expected {count} values, got {len(values)}")
    return np.array(values, dtype=float)


def load_gains():
    header = HEADER.read_text()
    return {
        "K_WHEEL_V": parse_gain(header, "K_WHEEL_V", 2),
        "K_GYRO": parse_gain(header, "K_GYRO", 3),
        "K_WHEEL_OMEGA": parse_gain(header, "K_WHEEL_OMEGA", 3),
        "K_MAG": parse_gain(header, "K_MAG", 3),
    }


def predict_matrix():
    A = np.eye(5)
    A[V, BA] = -DT
    A[THETA, OMEGA] = DT
    return A


def transitions(g):
    H_gyro = np.array([0.0, 0.0, 0.0, 1.0, 1.0])
    H_wheel_v = np.array([1.0, 0.0, 0.0, 0.0, 0.0])
    H_wheel_omega = np.array([0.0, 0.0, 0.0, 1.0, 0.0])
    H_mag = np.array([0.0, 0.0, 1.0, 0.0, 0.0])

    K_gyro = np.array([0.0, 0.0, g["K_GYRO"][0], g["K_GYRO"][1],
                       g["K_GYRO"][2]])
    K_wheel_v = np.array([g["K_WHEEL_V"][0], g["K_WHEEL_V"][1], 0.0, 0.0,
                          0.0])
    K_wheel_omega = np.array([0.0, 0.0, g["K_WHEEL_OMEGA"][0],
                              g["K_WHEEL_OMEGA"][1], g["K_WHEEL_OMEGA"][2]])
    K_mag = np.array([0.0, 0.0, g["K_MAG"][0], g["K_MAG"][1],
                      g["K_MAG"][2]])

    return {
        "gyro": (K_gyro, H_gyro),
        "wheel_v": (K_wheel_v, H_wheel_v),
        "wheel_omega": (K_wheel_omega, H_wheel_omega),
        "mag": (K_mag, H_mag),
    }


def update_matrix(K, H):
    return np.eye(5) - np.outer(K, H)


def cycle_transition(g, steps=100):
    A = predict_matrix()
    u = transitions(g)

    M = np.eye(5)
    for k in range(steps):
        M = A @ M
        M = update_matrix(*u["gyro"]) @ M
        if k % WHEEL_DIVIDER == 0:
            M = update_matrix(*u["wheel_v"]) @ M
            M = update_matrix(*u["wheel_omega"]) @ M
        if k % MAG_DIVIDER == 0:
            M = update_matrix(*u["mag"]) @ M
    return M


def wrap(a):
    a = math.fmod(a, 2.0 * math.pi)
    if a <= -math.pi:
        a += 2.0 * math.pi
    elif a > math.pi:
        a -= 2.0 * math.pi
    return a


def simulate(g, seconds=5.0):
    """Deterministic scenario with a velocity ramp, yaw rate, and biases."""
    steps = int(seconds / DT)
    v = 0.0
    b_a = 0.0
    theta = 0.0
    omega = 0.0
    b_g = 0.0

    v_true = 0.0
    omega_true = 0.2
    b_a_true = 0.05
    b_g_true = 0.01

    for i in range(steps):
        t = i * DT
        a_true = 0.2 if t < 1.0 else 0.0
        v_true += a_true * DT

        # Predict. The accelerometer reads a_true + b_a_true.
        v += (a_true + b_a_true - b_a) * DT
        theta = wrap(theta + omega * DT)

        # Gyro every step: omega + b_g.
        innov = (omega_true + b_g_true) - (omega + b_g)
        theta = wrap(theta + g["K_GYRO"][0] * innov)
        omega += g["K_GYRO"][1] * innov
        b_g += g["K_GYRO"][2] * innov

        # Wheel v/omega every 10 ms.
        if i % WHEEL_DIVIDER == 0:
            innov = v_true - v
            v += g["K_WHEEL_V"][0] * innov
            b_a += g["K_WHEEL_V"][1] * innov

            innov = omega_true - omega
            theta = wrap(theta + g["K_WHEEL_OMEGA"][0] * innov)
            omega += g["K_WHEEL_OMEGA"][1] * innov
            b_g += g["K_WHEEL_OMEGA"][2] * innov

    return {
        "v": v,
        "v_true": v_true,
        "b_a": b_a,
        "b_a_true": b_a_true,
        "omega": omega,
        "omega_true": omega_true,
        "b_g": b_g,
        "b_g_true": b_g_true,
    }


def main():
    g = load_gains()
    failures = []

    for name, values in g.items():
        if not np.all(np.isfinite(values)):
            failures.append(f"{name} contains non-finite values")

    u = transitions(g)
    A = predict_matrix()
    print("Individual update closed-loop |eig(A - A K H)|:")
    for name, (K, H) in u.items():
        eig = np.abs(np.linalg.eigvals(A @ update_matrix(K, H)))
        print(f"  {name:12s}: " + " ".join(f"{x:.6f}" for x in np.sort(eig)))

    M = cycle_transition(g)
    eig = np.abs(np.linalg.eigvals(M))
    max_eig = float(np.max(eig))
    print("\n100-step cycle (1 kHz predict/gyro, 100 Hz wheel, 10 Hz mag):")
    print("  |eig|: " + " ".join(f"{x:.6f}" for x in np.sort(eig)))
    print(f"  max |eig| = {max_eig:.6f}")
    if max_eig >= 1.0:
        failures.append(f"cycle is not stable: max |eig| = {max_eig:.6f}")

    sim = simulate(g)
    print("\nDeterministic simulation:")
    print(f"  v     = {sim['v']:.6f}  (true {sim['v_true']:.6f})")
    print(f"  b_a   = {sim['b_a']:.6f}  (true {sim['b_a_true']:.6f})")
    print(f"  omega = {sim['omega']:.6f}  (true {sim['omega_true']:.6f})")
    print(f"  b_g   = {sim['b_g']:.6f}  (true {sim['b_g_true']:.6f})")

    if abs(sim["v"] - sim["v_true"]) > 1e-2:
        failures.append("v did not track the velocity reference")
    if abs(sim["omega"] - sim["omega_true"]) > 1e-2:
        failures.append("omega did not track the yaw-rate reference")
    if abs(sim["b_g"] - sim["b_g_true"]) > 1e-2:
        failures.append("gyro bias did not converge")

    if failures:
        print("\nFAIL:")
        for failure in failures:
            print(f"  - {failure}")
        return 1

    print("\nPASS: gain bank is stable at the firmware update schedule.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
