#!/usr/bin/env python3
"""Step-by-step parity check: C++ filter vs. the NumPy reference.

Run with the host venv Python:

    tests/kalman_host/.venv/bin/python tests/kalman_host/test_parity.py

The script builds ``tests/kalman_host/parity.cpp`` together with
``src/src/kalman.cpp`` and runs it on a deterministic 3000-step scenario.
``parity.cpp`` is expected to have this interface:

    kalman-parity <scenario.csv> <output.csv>

It reads the scenario header written here::

    step,dt,a,gyro,wheel_v,wheel_omega,wheel_window,mag,use_gyro,use_wheel,use_mag

and, for every step, applies::

    predict(a, dt)
    if use_gyro:  updateGyro(gyro)
    if use_wheel: updateWheelV(wheel_v, wheel_window)
                  updateWheelOmega(wheel_omega, wheel_window)
    if use_mag:   updateMag(mag)

It writes one row per step with a header naming the 7 states, the 7
covariance diagonal entries and the magnetometer rejection flag.  Column
names such as ``x0..x6`` / ``s,theta,v,omega,b_a,b_g,b_m``, ``P00..P66`` /
``P0..P6`` and ``magRejected`` are accepted.

Exit codes: 0 = parity holds, 1 = build/run/mismatch failure, 2 = the C++
parity driver is not present yet.
"""

import csv
import importlib.util
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

N_STEPS = 3000
SEED = 7
DT = 0.001
WHEEL_WINDOW = 0.01
MAG_OUTLIER_STEP = 1500

STATE_NAMES = ["s", "theta", "v", "omega", "b_a", "b_g", "b_m"]
# C++ accumulates in float32, the reference in float64.  Over 3000 steps the
# difference is dominated by the nearly unobservable theta/b_m pair, so that
# pair gets a looser absolute floor; every other state stays at 1e-4 relative.
STATE_TOL = {
    "rtol": np.array([1e-4, 1e-3, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4]),
    "atol": np.array([1e-6, 1e-4, 1e-6, 1e-6, 1e-6, 1e-6, 5e-5]),
}
P_TOL = {"rtol": 1e-3, "atol": 1e-9}

SCENARIO_HEADER = [
    "step", "dt", "a", "gyro", "wheel_v", "wheel_omega", "wheel_window",
    "mag", "use_gyro", "use_wheel", "use_mag",
]

# Datasheet-derived sigmas, duplicated here on purpose: the scenario must not
# depend on the module under test.
SIGMA_ACCEL = 300e-6 * 9.80665 * np.sqrt(94.0)
SIGMA_GYRO = 0.1 * np.pi / 180.0
SIGMA_ANGLE = 0.068 * np.pi / 180.0
SIGMA_MAG = 0.01


def _wrap(a):
    """Fold into (-pi, pi], mirroring the C ``fmodf`` implementation."""
    a = np.fmod(a, 2.0 * np.pi)
    if a <= -np.pi:
        a += 2.0 * np.pi
    elif a > np.pi:
        a -= 2.0 * np.pi
    return a


def _g(value):
    return f"{value:.9g}"


def build_scenario(path, n=N_STEPS, seed=SEED):
    """Write the deterministic scenario CSV and return nothing."""
    rng = np.random.default_rng(seed)
    dt = np.full(n, DT)
    t = np.arange(n) * DT

    v = 0.2 + 0.05 * np.sin(2.0 * np.pi * 0.2 * t)
    omega = 0.1 * np.sin(2.0 * np.pi * 0.05 * t)
    theta = np.cumsum(omega * dt)

    a_true = np.zeros(n)
    a_true[1:] = np.diff(v) / dt[1:]

    b_a_true = 0.05
    b_g_true = 0.01
    b_m_true = 0.10

    a_meas = a_true + b_a_true + rng.normal(0.0, SIGMA_ACCEL, n)
    gyro = omega + b_g_true + rng.normal(0.0, SIGMA_GYRO, n)
    mag = np.array([_wrap(x) for x in
                    theta + b_m_true + rng.normal(0.0, SIGMA_MAG, n)])

    # Endpoint angle difference: sqrt(2) times the per-sample angle noise.
    sigma_wheel_v = (np.sqrt(2.0) * 0.0315 * SIGMA_ANGLE /
                     (9.6 * WHEEL_WINDOW))
    sigma_wheel_omega = (np.sqrt(2.0) * 2.0 * 0.0315 * SIGMA_ANGLE /
                         (0.14 * 9.6 * WHEEL_WINDOW))
    wheel_v = v + rng.normal(0.0, sigma_wheel_v, n)
    wheel_omega = omega + rng.normal(0.0, sigma_wheel_omega, n)
    wheel_window = np.full(n, WHEEL_WINDOW)

    use_gyro = np.ones(n, dtype=int)
    use_wheel = (np.arange(n) % 10 == 0).astype(int)
    use_mag = (np.arange(n) % 100 == 0).astype(int)

    # One 180 degree magnetometer outlier, still flagged valid.
    if MAG_OUTLIER_STEP < n:
        assert use_mag[MAG_OUTLIER_STEP] == 1
        mag[MAG_OUTLIER_STEP] = _wrap(mag[MAG_OUTLIER_STEP] + np.pi)

    with open(path, "w", newline="") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(SCENARIO_HEADER)
        for i in range(n):
            writer.writerow([
                i, _g(dt[i]), _g(a_meas[i]), _g(gyro[i]), _g(wheel_v[i]),
                _g(wheel_omega[i]), _g(wheel_window[i]), _g(mag[i]),
                int(use_gyro[i]), int(use_wheel[i]), int(use_mag[i]),
            ])


def read_scenario(path):
    """Read the scenario back into a list of plain-Python row dicts."""
    rows = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            rows.append({
                "dt": float(row["dt"]),
                "a": float(row["a"]),
                "gyro": float(row["gyro"]),
                "wheel_v": float(row["wheel_v"]),
                "wheel_omega": float(row["wheel_omega"]),
                "wheel_window": float(row["wheel_window"]),
                "mag": float(row["mag"]),
                "use_gyro": int(float(row["use_gyro"])),
                "use_wheel": int(float(row["use_wheel"])),
                "use_mag": int(float(row["use_mag"])),
            })
    return rows


def _norm(name):
    return name.strip().lower().replace("_", "").replace(" ", "").replace(
        "-", "")


def load_cpp_output(path, n_steps):
    """Parse the driver output into (states, P diagonals, rejections)."""
    with open(path, newline="") as f:
        raw = [row for row in csv.reader(f) if row]
    if not raw:
        raise RuntimeError(f"{path} is empty")

    header = raw[0]
    try:
        float(header[0])
        has_header = False
    except ValueError:
        has_header = True

    if not has_header:
        raise RuntimeError(
            "cpp output has no header row; expected columns for x0..x6, "
            "P00..P66 and magRejected")

    names = [_norm(h) for h in header]
    body = raw[1:]

    def find(*candidates):
        for candidate in candidates:
            if candidate in names:
                return names.index(candidate)
        return None

    state_cols = []
    for i, name in enumerate(STATE_NAMES):
        col = find(f"x{i}", f"state{i}", _norm(name))
        if col is None:
            raise RuntimeError(
                f"cpp output header lacks state {i} ({name}); header={header}")
        state_cols.append(col)

    p_cols = []
    for i, name in enumerate(STATE_NAMES):
        col = find(f"p{i}{i}", f"p{i}", f"diag{i}", f"cov{i}",
                   f"p{_norm(name)}")
        if col is None:
            raise RuntimeError(
                f"cpp output header lacks P[{i},{i}]; header={header}")
        p_cols.append(col)

    flag_col = find("magrejected", "magreject", "rejected", "magoutlier")
    if flag_col is None:
        raise RuntimeError(
            f"cpp output header lacks magRejected; header={header}")

    step_col = find("step", "index", "i")

    states = np.zeros((n_steps, 7))
    p_diag = np.zeros((n_steps, 7))
    rejected = np.zeros(n_steps, dtype=bool)

    if len(body) != n_steps:
        raise RuntimeError(
            f"cpp output has {len(body)} data rows, expected {n_steps}")

    for i, row in enumerate(body):
        if step_col is not None and int(float(row[step_col])) != i:
            raise RuntimeError(
                f"cpp output step mismatch at row {i}: "
                f"step={row[step_col]}")
        for j, col in enumerate(state_cols):
            states[i, j] = float(row[col])
        for j, col in enumerate(p_cols):
            p_diag[i, j] = float(row[col])
        rejected[i] = bool(int(float(row[flag_col])))

    return states, p_diag, rejected


def load_reference(root):
    """Import src/references/main.py by path, without sys.path changes."""
    path = root / "src" / "references" / "main.py"
    spec = importlib.util.spec_from_file_location("kalman_reference", path)
    module = importlib.util.module_from_spec(spec)
    # Keep the working tree clean: do not drop __pycache__ next to the source.
    previous = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    try:
        spec.loader.exec_module(module)
    finally:
        sys.dont_write_bytecode = previous
    return module


def replay(reference, rows):
    """Replay the scenario through the NumPy reference filter."""
    n = len(rows)
    kf = reference.DiffDriveKF()
    kf.reset(0.0)

    states = np.zeros((n, 7))
    p_diag = np.zeros((n, 7))
    rejected = np.zeros(n, dtype=bool)

    for i, row in enumerate(rows):
        kf.predict(row["a"], row["dt"])

        if row["use_gyro"]:
            kf.update_gyro(row["gyro"])

        if row["use_wheel"]:
            kf.update_wheel_v(row["wheel_v"], row["wheel_window"])
            kf.update_wheel_omega(row["wheel_omega"], row["wheel_window"])

        if row["use_mag"]:
            kf.update_mag(row["mag"])

        states[i] = kf.state
        p_diag[i] = np.diag(kf.covariance)
        rejected[i] = bool(kf.mag_rejected)

    return states, p_diag, rejected


def compare_states(reference, cpp):
    diff = np.abs(cpp - reference)
    allowed = STATE_TOL["atol"] + STATE_TOL["rtol"] * np.abs(reference)
    ok = bool(np.all(diff <= allowed))
    if not ok:
        ratio = diff / np.where(allowed > 0.0, allowed, 1.0)
        step, state = np.unravel_index(np.argmax(ratio), diff.shape)
        print(f"FAIL states: worst step {step}, state "
              f"{STATE_NAMES[state]} (x{state}): "
              f"python={reference[step, state]:.9g} "
              f"cpp={cpp[step, state]:.9g} "
              f"|delta|={diff[step, state]:.3e} "
              f"allowed={allowed[step, state]:.3e}")
    return ok, float(diff.max())


def compare_p(reference, cpp):
    diff = np.abs(cpp - reference)
    allowed = P_TOL["atol"] + P_TOL["rtol"] * np.abs(reference)
    ok = bool(np.all(diff <= allowed))
    if not ok:
        ratio = diff / np.where(allowed > 0.0, allowed, 1.0)
        step, state = np.unravel_index(np.argmax(ratio), diff.shape)
        print(f"FAIL P diagonal: worst step {step}, P[{state},{state}]: "
              f"python={reference[step, state]:.9e} "
              f"cpp={cpp[step, state]:.9e} "
              f"|delta|={diff[step, state]:.3e} "
              f"allowed={allowed[step, state]:.3e}")
    return ok, float(diff.max())


def main():
    root = Path(__file__).resolve().parents[2]
    parity_cpp = root / "tests" / "kalman_host" / "parity.cpp"
    kalman_cpp = root / "src" / "src" / "kalman.cpp"

    if not parity_cpp.is_file():
        print(f"SKIP: C++ parity driver not found at {parity_cpp}")
        print("      (another agent owns tests/kalman_host/parity.cpp)")
        return 2
    if not kalman_cpp.is_file():
        print(f"SKIP: C++ filter not found at {kalman_cpp}")
        return 2

    compiler = shutil.which("c++")
    if compiler is None:
        print("SKIP: no C++ compiler (c++) found on PATH")
        return 2

    with tempfile.TemporaryDirectory(prefix="kalman-parity-") as tmp:
        tmp = Path(tmp)
        exe = tmp / "kalman-parity"
        build = subprocess.run(
            [compiler, "-std=c++14", "-O2", "-Isrc/src", str(parity_cpp),
             str(kalman_cpp), "-o", str(exe)],
            cwd=root, text=True, capture_output=True,
        )
        if build.returncode != 0:
            print("FAIL: could not build the C++ parity driver")
            print(build.stdout)
            print(build.stderr)
            return 1

        scenario = tmp / "scenario.csv"
        build_scenario(scenario)
        cpp_csv = tmp / "cpp.csv"

        run = subprocess.run([str(exe), str(scenario), str(cpp_csv)],
                             cwd=root, text=True, capture_output=True)
        if run.returncode != 0:
            print(f"FAIL: parity driver exited with {run.returncode}")
            print(run.stdout)
            print(run.stderr)
            return 1
        if not cpp_csv.is_file() and run.stdout.strip():
            cpp_csv.write_text(run.stdout)
        if not cpp_csv.is_file():
            print(f"FAIL: parity driver did not write {cpp_csv.name}")
            print(run.stdout)
            print(run.stderr)
            return 1

        rows = read_scenario(scenario)
        cpp_states, cpp_p, cpp_rejected = load_cpp_output(
            cpp_csv, len(rows))

        reference = load_reference(root)
        py_states, py_p, py_rejected = replay(reference, rows)

        states_ok, max_state = compare_states(py_states, cpp_states)
        p_ok, max_p = compare_p(py_p, cpp_p)
        rejections_ok = bool(np.array_equal(py_rejected, cpp_rejected))
        if not rejections_ok:
            bad = np.flatnonzero(py_rejected != cpp_rejected)
            print(f"FAIL magRejected: {len(bad)} mismatching steps, first at "
                  f"{int(bad[0])} (python={py_rejected[bad[0]]}, "
                  f"cpp={cpp_rejected[bad[0]]})")
            print(f"     rejected steps python={int(py_rejected.sum())} "
                  f"cpp={int(cpp_rejected.sum())}")

        print(f"steps: {len(rows)}")
        print(f"max |state delta|:    {max_state:.3e}")
        print(f"max |P diagonal delta|: {max_p:.3e}")
        print(f"magRejected: python={int(py_rejected.sum())} "
              f"cpp={int(cpp_rejected.sum())} "
              f"({'match' if rejections_ok else 'MISMATCH'})")
        print("max |delta| per state:")
        for j, name in enumerate(STATE_NAMES):
            print(f"  {name:5s} {np.abs(py_states[:, j] - cpp_states[:, j]).max():.3e}")

        if states_ok and p_ok and rejections_ok:
            print("PASS: C++ and NumPy reference agree")
            return 0

        if not states_ok:
            diff = np.abs(cpp_states - py_states)
            allowed = (STATE_TOL["atol"] +
                       STATE_TOL["rtol"] * np.abs(py_states))
            bad_states = set(np.flatnonzero(
                np.any(diff > allowed, axis=0)).tolist())
            if bad_states and bad_states <= {1, 6}:
                print("note: only theta/b_m exceed the state tolerance; "
                      "float32 accumulation in the C++ filter shows up in "
                      "this nearly unobservable heading/bias pair.")
        print("FAIL: parity check failed")
        return 1


if __name__ == "__main__":
    sys.exit(main())
