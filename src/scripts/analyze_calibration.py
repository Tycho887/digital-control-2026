#!/usr/bin/env python3
"""Analyze a firmware calibration capture (`cal mag`, `cal noise`, `cal track`).

The firmware prints a `% CAL <mode>` line followed by the column header and
rows documented in `printCalLog()` (src/src/basebot_6.ino). With no log
argument the newest file in ../logs is used.

Examples:
    python3 src/scripts/analyze_calibration.py logs/device-monitor-....log
    python3 src/scripts/analyze_calibration.py --outdir /tmp/cal-plots
"""

import argparse
import sys
import warnings
from pathlib import Path

import numpy as np

# Geometry and assumed sensor noise (keep in sync with kalman.h / kalman.cpp).
GEAR = 9.6
WHEEL_RADIUS = 0.0315
WHEEL_BASE = 0.14
CONTROL_VOLTAGE_LIMIT = 3.0
SIGMA_GYRO_DEG = 0.1                  # deg/s
SIGMA_ACCEL = 0.0285                  # m/s^2
SIGMA_ANGLE_DEG = 0.068               # deg
SIGMA_ANGLE = np.deg2rad(SIGMA_ANGLE_DEG)
SIGMA_MAG = 0.01                      # rad
WHEEL_WINDOW_NOMINAL = 0.010          # s

# Column layout printed by printCalLog().
COL_TIME = 0
COL_MODE = 1
COL_FLAGS = 2
COL_KFMASK = 3
COL_SETPOINT = 4
COL_FEEDBACK = 6
COL_MOTORVOLT = 8
COL_MOTORVEL = 10
COL_GYRO = 12
COL_ACC = 15
COL_MAG = 18
COL_AS5ANGLE = 21
COL_AS5VEL = 23
COL_WINDOW = 25
COL_MAGHEADING = 26
COL_KFV = 27
COL_KFOMEGA = 28
COL_KFTHETA = 29
COL_KFINNOV = 30
COL_KFINNOVVAR = 34
N_COLUMNS = 38

FLAG_ACCEL_FRESH = 1 << 0
FLAG_GYRO_FRESH = 1 << 1
FLAG_MAG_FRESH = 1 << 2
FLAG_MAG_VALID = 1 << 3
FLAG_MAG_AVAILABLE = 1 << 4
FLAG_AS5_VALID = 1 << 5
FLAG_AS5_FRESH = 1 << 6
FLAG_KF_HEALTHY = 1 << 7

KF_GYRO = 1 << 0
KF_WHEEL_V = 1 << 1
KF_WHEEL_OMEGA = 1 << 2
KF_MAG = 1 << 3
KF_MAG_REJECTED = 1 << 4

STATE_NAMES = ["s", "theta", "v", "omega", "b_a", "b_g", "b_m"]


def latest_log(directory):
    logs = sorted(directory.glob("*.log"), key=lambda p: p.stat().st_mtime)
    if not logs:
        raise SystemExit("No .log files found in %s" % directory)
    return logs[-1]


def log_mode(path):
    with open(path, "r", errors="replace") as f:
        for line in f:
            if line.startswith("% CAL "):
                return line.split()[2]
    return None


def load_cal(path):
    path = Path(path)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        data = np.loadtxt(path, comments="%", ndmin=2)
    if data.size == 0:
        raise SystemExit("%s contains no data rows (only header/comments)." % path)
    if data.shape[1] < N_COLUMNS:
        raise SystemExit(
            "%s has %d columns; a calibration capture needs %d. "
            "This looks like a standard KF log, not a `cal` capture."
            % (path, data.shape[1], N_COLUMNS)
        )
    mode = log_mode(path)
    if mode is None:
        raise SystemExit("%s has no '%% CAL <mode>' header line." % path)
    return data, mode


def _bool_mask(data, bit):
    return (data[:, COL_FLAGS].astype(np.uint16) & bit) != 0


def _select(data, mask):
    if mask.any():
        return data[mask]
    return data


def _detrend(t, x):
    x = np.asarray(x, dtype=float)
    if x.size < 3:
        return x - np.mean(x) if x.size else x
    coeff = np.polyfit(t - t[0], x, 1)
    return x - np.polyval(coeff, t - t[0])


def _circular_span_rad(x):
    """Range of an angle sequence after unwrapping, in radians."""
    if x.size < 2:
        return 0.0
    return float(np.nanmax(np.unwrap(x)) - np.nanmin(np.unwrap(x)))


def _gyro_yaw(t, data):
    """Integrate the firmware's yaw signal with a static-bias correction."""
    gyro = data[:, COL_GYRO:COL_GYRO + 3]
    yaw_rate = -gyro[:, 0] * np.pi / 180.0
    dt = np.diff(t, prepend=t[0])
    dt[dt <= 0] = np.median(dt[dt > 0]) if np.any(dt > 0) else 0.001
    # A hand rotation usually has static segments before and after. Do not
    # use the median of the whole record: if rotation dominates it is the
    # rotation rate, not the bias.
    scale = float(np.percentile(np.abs(yaw_rate), 90)) if yaw_rate.size else 0.0
    threshold = max(0.05, 0.1 * scale)
    static = np.abs(yaw_rate) < threshold
    bias = float(np.median(yaw_rate[static])) if static.any() else 0.0
    active = np.abs(yaw_rate - bias) > threshold
    yaw = np.cumsum((yaw_rate - bias) * dt)
    first = np.flatnonzero(active)
    span = (int(first[0]), int(first[-1])) if first.size else (0, len(t) - 1)
    return yaw, yaw_rate, span


def _fit_heading(yaw, heading):
    """Fit heading = slope*yaw + intercept and return (slope, rmse)."""
    h = np.unwrap(np.asarray(heading, dtype=float))
    good = np.isfinite(h) & np.isfinite(yaw)
    if good.sum() < 5:
        return float("nan"), float("nan")
    A = np.vstack([yaw[good], np.ones(good.sum())]).T
    slope, intercept = np.linalg.lstsq(A, h[good], rcond=None)[0]
    residual = h[good] - (slope * yaw[good] + intercept)
    return float(slope), float(np.sqrt(np.mean(residual ** 2)))


def analyze_mag(t, data, outdir=None, show=False):
    """Identify the gravity/field axis and the firmware heading alignment."""
    acc_rows = _select(data, _bool_mask(data, FLAG_ACCEL_FRESH))
    acc_mean = np.mean(acc_rows[:, COL_ACC:COL_ACC + 3], axis=0)
    gravity_axis = int(np.argmax(np.abs(acc_mean)))

    mag_fresh = _bool_mask(data, FLAG_MAG_FRESH)
    mag_valid = _bool_mask(data, FLAG_MAG_VALID)
    mag_rows = _select(data, mag_fresh)
    mag = mag_rows[:, COL_MAG:COL_MAG + 3]
    mag_std = np.std(mag, axis=0)
    mag_ptp = np.ptp(mag, axis=0)
    vertical_axis = int(np.argmin(mag_std))

    yaw, yaw_rate, span = _gyro_yaw(t, data)
    i0, i1 = span
    margin = max(5, len(t) // 100)
    i0 = max(0, i0 - margin)
    i1 = min(len(t) - 1, i1 + margin)
    yaw_fit = yaw[i0:i1 + 1]

    candidates = []
    for i in range(3):
        for j in range(3):
            if i == j:
                continue
            for si in (1.0, -1.0):
                for sj in (1.0, -1.0):
                    heading = np.arctan2(si * data[:, COL_MAG + i],
                                         sj * data[:, COL_MAG + j])
                    slope, rmse = _fit_heading(yaw_fit, heading[i0:i1 + 1])
                    candidates.append({
                        "axes": (i, j), "signs": (si, sj),
                        "slope": slope, "rmse": rmse,
                    })
    candidates.sort(key=lambda c: (not np.isfinite(c["rmse"]), c["rmse"]))
    best = candidates[0]

    firmware = next(
        (c for c in candidates
         if c["axes"] == (1, 2) and c["signs"] == (1.0, -1.0)),
        None,
    )
    fw_slope, fw_rmse = _fit_heading(yaw_fit,
                                     data[i0:i1 + 1, COL_MAGHEADING])
    fw_ok = bool(np.isfinite(fw_slope) and abs(fw_slope - 1.0) < 0.25
                 and fw_rmse < 0.20)

    result = {
        "mode": "mag",
        "mag_available": bool(_bool_mask(data, FLAG_MAG_AVAILABLE).any()
                              or mag_valid.any()),
        "mag_valid_samples": int(mag_valid.sum()),
        "gravity_axis": gravity_axis,
        "gravity_mean": acc_mean,
        "vertical_axis": vertical_axis,
        "mag_std": mag_std,
        "mag_ptp": mag_ptp,
        "firmware_slope": fw_slope,
        "firmware_rmse": fw_rmse,
        "firmware_ok": fw_ok,
        "firmware_pair": firmware,
        "best": best,
        "candidates": candidates,
    }

    print("\n=== Magnetometer axis check ===")
    print("mag valid samples: %d" % result["mag_valid_samples"])
    print("accelerometer mean (g): x=%+.3f y=%+.3f z=%+.3f"
          % tuple(acc_mean))
    print("gravity axis: %s" % "XYZ"[gravity_axis])
    print("mag component sigma (counts): x=%.2f y=%.2f z=%.2f"
          % tuple(mag_std))
    print("mag component peak-to-peak (counts): x=%.2f y=%.2f z=%.2f"
          % tuple(mag_ptp))
    print("smallest-variation (vertical) field axis: %s"
          % "XYZ"[vertical_axis])
    if not result["mag_available"]:
        print("MAGNETOMETER UNAVAILABLE: check `cal retrymag` output and the "
              "AK8963 wiring; alignment cannot be verified.")
    else:
        print("firmware heading atan2(Y, -Z): slope vs gyro = %.3f, RMSE = "
              "%.3f rad" % (fw_slope, fw_rmse))
        print("best candidate: atan2(%+.0f*%s, %+.0f*%s), slope = %.3f, "
              "RMSE = %.3f rad"
              % (best["signs"][0], "XYZ"[best["axes"][0]],
                 best["signs"][1], "XYZ"[best["axes"][1]],
                 best["slope"], best["rmse"]))
        print("verdict: %s" % ("aligned" if fw_ok else "CHECK AXIS/SIGN"))
        if not fw_ok and abs(best["slope"]) > 0.5:
            print("          best slope is %+.2f; a negative slope means the "
                  "heading sign is reversed." % best["slope"])
        if vertical_axis != 0 or gravity_axis != 0:
            print("NOTE: the vertical axis is not X (gravity=%s, field=%s). "
                  "Update imuYawAxis, imuForwardAxis and the magnetometer "
                  "axis constants together." %
                  ("XYZ"[gravity_axis], "XYZ"[vertical_axis]))

    if outdir is not None or show:
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        axes[0].plot(t, data[:, COL_MAG], lw=0.8)
        axes[0].set_ylabel("mag (counts)")
        axes[0].legend(["x", "y", "z"], loc="best")
        axes[0].set_title("Raw magnetometer components")
        axes[0].grid(True, alpha=0.3)
        axes[1].plot(t, np.unwrap(data[:, COL_MAGHEADING]), lw=1.0,
                     label="firmware heading")
        axes[1].plot(t, yaw, lw=1.0, label="gyro-integrated yaw")
        axes[1].set_xlabel("time (s)")
        axes[1].set_ylabel("angle (rad)")
        axes[1].legend(loc="best")
        axes[1].grid(True, alpha=0.3)
        fig.tight_layout()
        _finish_fig(fig, outdir, "cal_mag")

    return result


def _sigma_wheel_v(window):
    return (np.sqrt(2.0) * WHEEL_RADIUS * SIGMA_ANGLE /
            (GEAR * window))


def _sigma_wheel_omega(window):
    return (np.sqrt(2.0) * 2.0 * WHEEL_RADIUS * SIGMA_ANGLE /
            (WHEEL_BASE * GEAR * window))


def _nis_table(data, mask_bit, label, index):
    mask = (data[:, COL_KFMASK].astype(np.uint16) & mask_bit) != 0
    rows = data[mask]
    if rows.shape[0] == 0:
        return None
    innov = rows[:, COL_KFINNOV + index]
    s = rows[:, COL_KFINNOVVAR + index]
    good = np.isfinite(innov) & np.isfinite(s) & (s > 0)
    if good.sum() == 0:
        return None
    nis = innov[good] ** 2 / s[good]
    return {
        "label": label,
        "n": int(good.sum()),
        "mean": float(np.mean(nis)),
        "median": float(np.median(nis)),
        "p95": float(np.percentile(nis, 95)),
        "frac_gt_3.84": float(np.mean(nis > 3.84)),
    }


def analyze_noise(t, data, outdir=None, show=False):
    """Measure per-sensor sigma and KF normalized innovation squared."""
    gyro_rows = _select(data, _bool_mask(data, FLAG_GYRO_FRESH))
    gyro_sigma_deg = [
        np.std(_detrend(gyro_rows[:, COL_TIME], gyro_rows[:, COL_GYRO + k]))
        for k in range(3)
    ]

    acc_rows = _select(data, _bool_mask(data, FLAG_ACCEL_FRESH))
    acc_g = acc_rows[:, COL_ACC:COL_ACC + 3]
    acc_sigma = [
        np.std(_detrend(acc_rows[:, COL_TIME], acc_g[:, k])) * 9.80665
        for k in range(3)
    ]

    as5_mask = _bool_mask(data, FLAG_AS5_VALID)
    as5_rows = _select(data, as5_mask)
    angle_l = np.unwrap(as5_rows[:, COL_AS5ANGLE])
    angle_r = np.unwrap(as5_rows[:, COL_AS5ANGLE + 1])
    angle_sigma = 0.5 * (
        np.std(_detrend(as5_rows[:, COL_TIME], angle_l)) +
        np.std(_detrend(as5_rows[:, COL_TIME], angle_r))
    )

    fresh_mask = _bool_mask(data, FLAG_AS5_FRESH)
    vel_rows = _select(data, fresh_mask)
    if vel_rows.shape[0] < 4:
        vel_rows = as5_rows
    v_l = vel_rows[:, COL_AS5VEL]
    v_r = vel_rows[:, COL_AS5VEL + 1]
    window = float(np.median(vel_rows[:, COL_WINDOW]))
    if not np.isfinite(window) or window <= 0:
        window = WHEEL_WINDOW_NOMINAL
    v_sigma = 0.5 * (np.std(_detrend(vel_rows[:, COL_TIME], v_l)) +
                     np.std(_detrend(vel_rows[:, COL_TIME], v_r)))
    omega = (v_r - v_l) / WHEEL_BASE
    omega_sigma = np.std(_detrend(vel_rows[:, COL_TIME], omega))

    mag_mask = _bool_mask(data, FLAG_MAG_FRESH) & _bool_mask(data, FLAG_MAG_VALID)
    mag_rows = _select(data, mag_mask)
    mag_heading_sigma = float("nan")
    if mag_rows.shape[0] >= 4:
        mag_heading_sigma = np.std(
            _detrend(mag_rows[:, COL_TIME],
                     np.unwrap(mag_rows[:, COL_MAGHEADING]))
        )

    assumed_v = _sigma_wheel_v(window)
    assumed_omega = _sigma_wheel_omega(window)
    assume_angle = SIGMA_ANGLE

    print("\n=== Sensor noise check (1-sigma) ===")
    print("%-22s %12s %12s %8s  %s" %
          ("quantity", "assumed", "measured", "ratio", "verdict"))
    checks = [
        ("gyro yaw (deg/s)", SIGMA_GYRO_DEG, gyro_sigma_deg[0]),
        ("accel forward (m/s^2)", SIGMA_ACCEL, acc_sigma[2]),
        ("AS5X47 angle (rad)", assume_angle, angle_sigma),
        ("wheel v (m/s)", assumed_v, v_sigma),
        ("wheel omega (rad/s)", assumed_omega, omega_sigma),
        ("mag heading (rad)", SIGMA_MAG, mag_heading_sigma),
    ]
    results = {}
    for name, assumed, measured in checks:
        ratio = measured / assumed if assumed > 0 and np.isfinite(measured) else np.nan
        plausible = np.isfinite(ratio) and 0.5 <= ratio <= 2.0
        print("%-22s %12.4g %12.4g %8.3f  %s" %
              (name, assumed, measured, ratio,
               "plausible" if plausible else "CHECK R"))
        results[name] = {
            "assumed": float(assumed),
            "measured": float(measured),
            "ratio": float(ratio),
            "plausible": bool(plausible),
        }
    if not _bool_mask(data, FLAG_MAG_VALID).any():
        print("NOTE: no valid magnetometer heading in this capture; run "
              "`cal retrymag` and check the AK8963 output.")
    print("note: AS5X47 angle is a per-sample estimate; the filter uses a "
          "10 ms endpoint difference (window %.2f ms)." % (window * 1e3))
    print("gyro all axes (deg/s): x=%.4f y=%.4f z=%.4f" % tuple(gyro_sigma_deg))
    print("accel all axes (m/s^2): x=%.4f y=%.4f z=%.4f" % tuple(acc_sigma))

    print("\n=== KF innovation consistency (NIS = y^2 / S) ===")
    print("%-14s %6s %10s %10s %10s %10s" %
          ("update", "n", "mean", "median", "p95", "frac>3.84"))
    nis_rows = []
    for bit, label, index in ((KF_GYRO, "gyro", 0),
                              (KF_WHEEL_V, "wheel v", 1),
                              (KF_WHEEL_OMEGA, "wheel omega", 2),
                              (KF_MAG, "mag", 3),
                              (KF_MAG_REJECTED, "mag rejected", 3)):
        row = _nis_table(data, bit, label, index)
        if row is None:
            continue
        nis_rows.append(row)
        print("%-14s %6d %10.3f %10.3f %10.3f %10.3f" %
              (label, row["n"], row["mean"], row["median"],
               row["p95"], row["frac_gt_3.84"]))
    print("NIS ~ 1 is consistent; mean >> 1 means R too small or P too "
          "confident. The wheel v/omega rows share angle noise, so treat "
          "those two as approximate.")

    if outdir is not None or show:
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        axes[0].plot(t, data[:, COL_GYRO], lw=0.7)
        axes[0].set_ylabel("gyro (deg/s)")
        axes[0].legend(["x", "y", "z"], loc="best")
        axes[0].grid(True, alpha=0.3)
        axes[1].plot(t, data[:, COL_ACC], lw=0.7)
        axes[1].set_ylabel("accel (g)")
        axes[1].legend(["x", "y", "z"], loc="best")
        axes[1].grid(True, alpha=0.3)
        axes[2].plot(t, data[:, COL_MAGHEADING], lw=0.8)
        axes[2].set_ylabel("mag heading (rad)")
        axes[2].set_xlabel("time (s)")
        axes[2].grid(True, alpha=0.3)
        fig.tight_layout()
        _finish_fig(fig, outdir, "cal_noise")

    return {"checks": results, "nis": nis_rows}


def _step_metrics(t, y, target, baseline, u, start, end):
    y_seg = y[start:end]
    t_seg = t[start:end]
    delta = target - baseline
    direction = 1.0 if delta >= 0 else -1.0
    if abs(delta) < 1e-9 or t_seg.size < 3:
        return None

    frac = direction * (y_seg - baseline) / delta
    i10 = np.flatnonzero(frac >= 0.1)
    i90 = np.flatnonzero(frac >= 0.9)
    rise = float("nan")
    if i10.size and i90.size:
        rise = float(t_seg[i90[0]] - t_seg[i10[0]])

    overshoot = max(0.0, float(np.max(direction * (y_seg - target))) /
                    abs(delta))

    band = max(0.05 * abs(delta), 0.5)
    outside = np.flatnonzero(np.abs(y_seg - target) > band)
    settle = float("nan")
    if outside.size == 0:
        settle = 0.0
    elif outside[-1] + 1 < y_seg.size:
        settle = float(t_seg[outside[-1] + 1] - t_seg[0])

    tail = max(1, int(0.3 * y_seg.size))
    ss_error = float(np.mean(y_seg[-tail:] - target))
    rms = float(np.sqrt(np.mean((y_seg - target) ** 2)))
    sat = float(np.mean(np.abs(u[start:end]) >= 0.99 * CONTROL_VOLTAGE_LIMIT))
    return {
        "rise": rise,
        "overshoot": overshoot,
        "settle": settle,
        "ss_error": ss_error,
        "rms": rms,
        "saturation": sat,
        "n": int(y_seg.size),
    }


def _find_steps(setpoint, threshold=2.0):
    d = np.diff(setpoint)
    changes = np.flatnonzero(np.abs(d) > threshold)
    starts = []
    for c in changes:
        starts.append(c + 1)
    return starts


def analyze_track(t, data, outdir=None, show=False):
    """Step-response metrics for PI feedback and independent velocity."""
    sp = data[:, COL_SETPOINT:COL_SETPOINT + 2]
    feedback = data[:, COL_FEEDBACK:COL_FEEDBACK + 2]
    as5_motor = data[:, COL_AS5VEL:COL_AS5VEL + 2] * (GEAR / WHEEL_RADIUS)
    kf_motor = data[:, COL_KFV] * (GEAR / WHEEL_RADIUS)
    voltage = data[:, COL_MOTORVOLT:COL_MOTORVOLT + 2]

    metrics = []
    for wheel in range(2):
        starts = _find_steps(sp[:, wheel])
        if not starts:
            continue
        starts = [0] + starts
        for k, start in enumerate(starts):
            end = starts[k + 1] if k + 1 < len(starts) else len(t)
            if end - start < 5:
                continue
            target = float(np.median(sp[start:end, wheel]))
            prev_start = starts[k - 1] if k > 0 else 0
            prev_end = start
            if prev_end - prev_start < 3:
                baseline = float(feedback[0, wheel])
            else:
                baseline = float(np.median(
                    feedback[prev_end - max(2, (prev_end - prev_start) // 5):
                             prev_end, wheel]))
            if abs(target - baseline) < 2.0:
                continue
            for name, signal in (("feedback", feedback[:, wheel]),
                                 ("as5x47", as5_motor[:, wheel]),
                                 ("kf", kf_motor)):
                m = _step_metrics(t, signal, target, baseline,
                                  voltage[:, wheel], start, end)
                if m is None:
                    continue
                m.update({
                    "wheel": "L" if wheel == 0 else "R",
                    "signal": name,
                    "target": target,
                    "baseline": baseline,
                    "t0": float(t[start]),
                })
                metrics.append(m)

    print("\n=== PI setpoint tracking ===")
    if not metrics:
        print("No step changes found; check the `cal track` setpoint schedule.")
        return {"steps": [], "summary": {}}

    print("%-3s %-8s %8s %8s %9s %10s %10s %10s %7s" %
          ("wh", "signal", "target", "rise s", "oversh %", "settle s",
           "ss err", "rms err", "sat %"))
    for m in metrics:
        print("%-3s %-8s %8.1f %8.3f %9.1f %10s %10.3f %10.3f %7.1f" %
              (m["wheel"], m["signal"], m["target"], m["rise"],
               100.0 * m["overshoot"],
               ("%.3f" % m["settle"]) if np.isfinite(m["settle"]) else "n/a",
               m["ss_error"], m["rms"], 100.0 * m["saturation"]))

    summary = {}
    for name in ("feedback", "as5x47", "kf"):
        rows = [m for m in metrics if m["signal"] == name]
        if not rows:
            continue
        summary[name] = {
            "n": len(rows),
            "median_rise": float(np.nanmedian([m["rise"] for m in rows])),
            "median_overshoot": float(np.nanmedian(
                [m["overshoot"] for m in rows])),
            "median_abs_ss_error": float(np.nanmedian(
                [abs(m["ss_error"]) for m in rows])),
            "median_rms": float(np.nanmedian([m["rms"] for m in rows])),
            "mean_saturation": float(np.mean(
                [m["saturation"] for m in rows])),
        }
    print("\nSummary by signal:")
    for name, s in summary.items():
        print("  %-8s n=%d median rise=%.3f s overshoot=%.1f%% "
              "|SS err|=%.3f RMS=%.3f sat=%.1f%%" %
              (name, s["n"], s["median_rise"],
               100.0 * s["median_overshoot"], s["median_abs_ss_error"],
               s["median_rms"], 100.0 * s["mean_saturation"]))
    print("Compare feedback with as5x47/kf: a good feedback metric but a poor "
          "independent metric points at the encoder velocity estimate.")

    if outdir is not None or show:
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
        axes[0].plot(t, sp, lw=1.4, label=["setpoint L", "setpoint R"])
        axes[0].plot(t, feedback, lw=0.8, alpha=0.8,
                     label=["PI feedback L", "PI feedback R"])
        axes[0].plot(t, as5_motor, lw=0.8, ls="--",
                     label=["AS5X47 L", "AS5X47 R"])
        axes[0].set_ylabel("motor speed (rad/s)")
        axes[0].legend(loc="best", fontsize=7, ncol=2)
        axes[0].grid(True, alpha=0.3)
        axes[1].plot(t, sp[:, 0] - feedback[:, 0], lw=0.8, label="L")
        axes[1].plot(t, sp[:, 1] - feedback[:, 1], lw=0.8, label="R")
        axes[1].set_ylabel("setpoint - feedback (rad/s)")
        axes[1].legend(loc="best")
        axes[1].grid(True, alpha=0.3)
        axes[2].plot(t, voltage, lw=0.8)
        axes[2].axhline(CONTROL_VOLTAGE_LIMIT, color="k", ls=":", lw=0.8)
        axes[2].axhline(-CONTROL_VOLTAGE_LIMIT, color="k", ls=":", lw=0.8)
        axes[2].set_ylabel("motor voltage (V)")
        axes[2].set_xlabel("time (s)")
        axes[2].grid(True, alpha=0.3)
        fig.tight_layout()
        _finish_fig(fig, outdir, "cal_track")

    return {"steps": metrics, "summary": summary}


def _finish_fig(fig, outdir, stem):
    if outdir is not None:
        outdir = Path(outdir)
        outdir.mkdir(parents=True, exist_ok=True)
        path = outdir / (stem + ".png")
        fig.savefig(path, dpi=150)
        print("Saved %s" % path)
    else:
        import matplotlib.pyplot as plt
        plt.show()


def main():
    default_logs = Path(__file__).resolve().parents[1] / "logs"
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "log", nargs="?", type=Path,
        help="calibration log (default: newest in %s)" % default_logs,
    )
    parser.add_argument(
        "--outdir", type=Path,
        help="save figures as PNG in this directory instead of showing them",
    )
    parser.add_argument(
        "--no-plots", action="store_true",
        help="print the summary only",
    )
    args = parser.parse_args()

    path = args.log or latest_log(default_logs)
    data, mode = load_cal(path)
    t = data[:, COL_TIME]
    print("Log:  %s" % path)
    print("Mode: %s" % mode)
    print("Samples: %d over %.3f s" % (len(t), t[-1] - t[0]))

    show = not args.no_plots and args.outdir is None
    outdir = args.outdir
    if mode == "mag":
        analyze_mag(t, data, outdir=outdir, show=show)
    elif mode == "noise":
        analyze_noise(t, data, outdir=outdir, show=show)
    elif mode == "track":
        analyze_track(t, data, outdir=outdir, show=show)
    else:
        raise SystemExit("Unknown calibration mode %r" % mode)
    return 0


if __name__ == "__main__":
    sys.exit(main())
