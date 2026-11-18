#!/usr/bin/env python3
"""Verify the stationary gain bank and its DARE origin.

The firmware predicts at 1 kHz, applies the gyro update every step, and the
wheel v/omega updates every 10 ms. The gain bank is solved per observable
block at those rates. This script checks the 100-step closed-loop error
transition, simulates a deterministic scenario, and compares the header gains
with a NumPy replica of the DARE.

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

# State order used by the checks: [v, omega, b_a].
V, OMEGA, BA = range(3)

# DARE inputs, kept in sync with kalman_filter_tuning/solve_dare.m.
SIGMA_GYRO_DPS = 0.17
SIGMA_ACCEL = 0.12
SIGMA_WHEEL_V = 0.0145
SIGMA_WHEEL_OMEGA = 0.18
Q_V = 1.0
Q_OM = 0.7
Q_BA = 3e-3


def parse_gain(header, name, count=None):
    if count is None:
        match = re.search(rf"{name}\s*=\s*([-+0-9.eEfF]+)\s*;", header)
        if match is None:
            raise RuntimeError(f"could not find {name} in {HEADER}")
        return float(match.group(1).rstrip("fF"))
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
        "K_GYRO": np.array([parse_gain(header, "K_GYRO")]),
        "K_WHEEL_OMEGA": np.array([parse_gain(header, "K_WHEEL_OMEGA")]),
    }


def dare_gain(F, H, Q, R, it=50000, tol=1e-13):
    """Steady-state Kalman predictor DARE for x+ = F x + w, z = H x + v."""
    P = Q.copy()
    for _ in range(it):
        S = H @ P @ H.T + R
        Si = np.linalg.inv(S)
        Pn = F @ P @ F.T - F @ P @ H.T @ Si @ H @ P @ F.T + Q
        if np.max(np.abs(Pn - P)) < tol * max(1.0, np.max(np.abs(Pn))):
            P = Pn
            break
        P = Pn
    S = H @ P @ H.T + R
    return P @ H.T @ np.linalg.inv(S)


def expected_gains():
    """NumPy replica of kalman_filter_tuning/solve_dare.m."""
    sigma_gyro = SIGMA_GYRO_DPS * math.pi / 180.0

    F_t = np.array([[1.0, -DT], [0.0, 1.0]])
    B_t = np.array([[DT], [0.0]])
    Q_t = B_t * (SIGMA_ACCEL ** 2 + Q_V ** 2) @ B_t.T
    Q_t = Q_t + np.diag([0.0, Q_BA ** 2 * DT])
    F_tw = np.linalg.matrix_power(F_t, WHEEL_DIVIDER)
    Q_tw = np.zeros((2, 2))
    Ak = np.eye(2)
    for _ in range(WHEEL_DIVIDER):
        Q_tw += Ak @ Q_t @ Ak.T
        Ak = Ak @ F_t
    K_v = dare_gain(F_tw, np.array([[1.0, 0.0]]), Q_tw,
                    np.array([[SIGMA_WHEEL_V ** 2]]))

    # Shared periodic covariance: gyro each tick, then wheel at cycle end.
    rg, rw = sigma_gyro ** 2, SIGMA_WHEEL_OMEGA ** 2
    posterior = rg
    for _ in range(100000):
        previous = posterior
        gyro_cycle = []
        for tick in range(WHEEL_DIVIDER):
            prior = posterior + (Q_OM * DT) ** 2
            kg = prior / (prior + rg)
            gyro_cycle.append(kg)
            posterior = (1 - kg) * prior
            if tick == WHEEL_DIVIDER - 1:
                kw = posterior / (posterior + rw)
                posterior *= 1 - kw
        if abs(posterior - previous) < 1e-12 * max(rg, rw):
            break
    else:
        raise RuntimeError("Joint covariance did not converge")
    return K_v.ravel(), np.array([np.mean(gyro_cycle)]), np.array([kw])


def predict_matrix():
    A = np.eye(3)
    A[V, BA] = -DT
    return A


def transitions(g):
    H_gyro = np.array([0.0, 1.0, 0.0])
    H_wheel_v = np.array([1.0, 0.0, 0.0])
    H_wheel_omega = np.array([0.0, 1.0, 0.0])

    K_gyro = np.array([0.0, float(g["K_GYRO"][0]), 0.0])
    K_wheel_v = np.array([g["K_WHEEL_V"][0], 0.0, g["K_WHEEL_V"][1]])
    K_wheel_omega = np.array([0.0, float(g["K_WHEEL_OMEGA"][0]), 0.0])

    return {
        "gyro": (K_gyro, H_gyro),
        "wheel_v": (K_wheel_v, H_wheel_v),
        "wheel_omega": (K_wheel_omega, H_wheel_omega),
    }


def update_matrix(K, H):
    return np.eye(3) - np.outer(K, H)


def cycle_transition(g, steps=100):
    A = predict_matrix()
    u = transitions(g)

    M = np.eye(3)
    for k in range(steps):
        M = A @ M
        M = update_matrix(*u["gyro"]) @ M
        if k % WHEEL_DIVIDER == 0:
            M = update_matrix(*u["wheel_v"]) @ M
            M = update_matrix(*u["wheel_omega"]) @ M
    return M


def simulate(g, seconds=60.0):
    """Deterministic scenario with a velocity ramp, yaw rate, and bias."""
    steps = int(seconds / DT)
    v = 0.0
    b_a = 0.0
    omega = 0.0

    v_true = 0.0
    omega_true = 0.2
    b_a_true = 0.05

    for i in range(steps):
        t = i * DT
        a_true = 0.2 if t < 1.0 else 0.0
        v_true += a_true * DT

        # Predict. The accelerometer reads a_true + b_a_true.
        v += (a_true + b_a_true - b_a) * DT

        # Gyro every step.
        innov = omega_true - omega
        omega += float(g["K_GYRO"][0]) * innov

        # Wheel v/omega every 10 ms.
        if i % WHEEL_DIVIDER == 0:
            innov = v_true - v
            v += g["K_WHEEL_V"][0] * innov
            b_a += g["K_WHEEL_V"][1] * innov

            innov = omega_true - omega
            omega += float(g["K_WHEEL_OMEGA"][0]) * innov

    return {
        "v": v,
        "v_true": v_true,
        "b_a": b_a,
        "b_a_true": b_a_true,
        "omega": omega,
        "omega_true": omega_true,
    }


def main():
    g = load_gains()
    failures = []

    for name, values in g.items():
        if not np.all(np.isfinite(values)):
            failures.append(f"{name} contains non-finite values")

    expected_v, expected_g, expected_w = expected_gains()
    checks = (
        ("K_WHEEL_V", g["K_WHEEL_V"], expected_v),
        ("K_GYRO", g["K_GYRO"], expected_g),
        ("K_WHEEL_OMEGA", g["K_WHEEL_OMEGA"], expected_w),
    )
    print("DARE gain check (header vs NumPy replica):")
    for name, got, expected in checks:
        ok = np.allclose(got, expected, rtol=0.05, atol=1e-6)
        print(f"  {name:14s} header={np.array2string(got, precision=6)} "
              f"expected={np.array2string(expected, precision=6)} "
              f"{'OK' if ok else 'MISMATCH'}")
        if not ok:
            failures.append(f"{name} does not match the DARE within 5%")

    u = transitions(g)
    A = predict_matrix()
    print("\nIndividual update closed-loop |eig(A - A K H)|:")
    for name, (K, H) in u.items():
        eig = np.abs(np.linalg.eigvals(A @ update_matrix(K, H)))
        print(f"  {name:12s}: " + " ".join(f"{x:.6f}" for x in np.sort(eig)))

    M = cycle_transition(g)
    eig = np.abs(np.linalg.eigvals(M))
    max_eig = float(np.max(eig))
    print("\n100-step cycle (1 kHz predict/gyro, 100 Hz wheel):")
    print("  |eig|: " + " ".join(f"{x:.6f}" for x in np.sort(eig)))
    print(f"  max |eig| = {max_eig:.6f}")
    if max_eig >= 1.0:
        failures.append(f"cycle is not stable: max |eig| = {max_eig:.6f}")

    sim = simulate(g)
    print("\nDeterministic simulation:")
    print(f"  v   = {sim['v']:.6f}  (true {sim['v_true']:.6f})")
    print(f"  b_a = {sim['b_a']:.6f}  (true {sim['b_a_true']:.6f})")
    print(f"  omega = {sim['omega']:.6f}  (true {sim['omega_true']:.6f})")

    if abs(sim["v"] - sim["v_true"]) > 1e-2:
        failures.append("v did not track the velocity reference")
    if abs(sim["omega"] - sim["omega_true"]) > 1e-2:
        failures.append("omega did not track the yaw-rate reference")
    if abs(sim["b_a"] - sim["b_a_true"]) > 2e-3:
        failures.append("accelerometer bias did not converge")

    if failures:
        print("\nFAIL:")
        for failure in failures:
            print(f"  - {failure}")
        return 1

    print("\nPASS: gain bank matches the DARE and is stable at the update schedule.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
