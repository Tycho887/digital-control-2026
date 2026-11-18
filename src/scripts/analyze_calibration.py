#!/usr/bin/env python3
"""Analyze a firmware calibration capture.

Handles `cal mag`, `cal noise`, `cal track`, `cal yaw` and `cal upside`. The firmware
prints a `% CAL <mode>` line followed by the column header and rows documented
in `printCalLog()` (src/src/basebot_6.ino). With no log argument the newest
file in ../logs is used.

Examples:
    python3 src/scripts/analyze_calibration.py logs/device-monitor-....log
    python3 src/scripts/analyze_calibration.py --outdir /tmp/cal-plots
"""

import argparse
import sys
import warnings
from pathlib import Path

import numpy as np

# Geometry and assumed sensor noise (keep in sync with
# kalman_filter_tuning/solve_dare.m and the gain bank in kalman.h).
GEAR = 9.6
WHEEL_RADIUS = 0.0315
WHEEL_BASE = 0.14
CONTROL_VOLTAGE_LIMIT = 7.0
SIGMA_GYRO_DEG = 0.17                 # deg/s, DARE input from the 14 Nov logs
SIGMA_ACCEL = 0.12                    # m/s^2, in-motion DARE input
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
COL_KFBA = 30
COL_KFBG = 31
COL_KFINNOV = 32
COL_KFINNOVVAR = 36
COL_PHASE = 40
COL_MAGSTATUS = 41
COL_IMUDT = 42
COL_ENCODER = 43
N_COLUMNS = 40
N_COLUMNS_FULL = 45

# Firmware axis mapping and upside-down schedule (keep in sync with
# basebot_6.ino and kalman.h).
FORWARD_ACCEL_AXIS = 2       # imuForwardAxis
FORWARD_ACCEL_SIGN = -1.0    # imuForwardAccelSign
YAW_GYRO_AXIS = 0            # imuYawAxis
YAW_GYRO_SIGN = -1.0         # imuYawRateSign
SETTLE_EXCLUDE = 0.5         # s discarded after each phase transition
ENCODER_MOTION = 5.0         # motor rad/s, encoderMotion stall threshold
KF_WHEEL_V0 = 0.197604345    # kalman.h K_WHEEL_V[0]
PHASE_LABELS = {10: "baseline", 11: "forward", 12: "turn", 2: "settling"}
PHASE_ORDER = (10, 11, 12, 2)

FLAG_ACCEL_FRESH = 1 << 0
FLAG_GYRO_FRESH = 1 << 1
FLAG_MAG_FRESH = 1 << 2
FLAG_MAG_VALID = 1 << 3
FLAG_MAG_AVAILABLE = 1 << 4
FLAG_AS5_VALID = 1 << 5
FLAG_AS5_FRESH = 1 << 6
FLAG_KF_HEALTHY = 1 << 7
CAL_MAG_ATTEMPT = 1 << 8
CAL_MAG_DRDY = 1 << 9
CAL_MAG_OVERFLOW = 1 << 10

KF_GYRO = 1 << 0
KF_WHEEL_V = 1 << 1
KF_WHEEL_OMEGA = 1 << 2
KF_WHEEL_ENCODER = 1 << 5

STATE_NAMES = ["v", "omega", "theta", "b_a", "reserved"]


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
    if data.shape[1] >= 55 and bit in (FLAG_GYRO_FRESH, FLAG_ACCEL_FRESH):
        col = 51 if bit == FLAG_GYRO_FRESH else 53
        return (data[:, col] != 0) & np.r_[True, np.diff(data[:, col]) != 0]
    return (data[:, COL_FLAGS].astype(np.uint16) & bit) != 0


def capture_status(data):
    """Optional timing and AK8963 status from the 45-column captures.

    Older 40/41-column logs return an empty dict.
    """
    if data.shape[1] < N_COLUMNS_FULL:
        return {}
    flags = data[:, COL_FLAGS].astype(np.uint16)
    dt = data[:, COL_IMUDT].astype(float)
    encoder = data[:, COL_ENCODER:COL_ENCODER + 2]
    return {
        "imu_dt_us": {
            "median": float(np.median(dt)),
            "min": float(np.min(dt)),
            "max": float(np.max(dt)),
        },
        "mag": {
            "attempt": int(np.sum((flags & CAL_MAG_ATTEMPT) != 0)),
            "drdy": int(np.sum((flags & CAL_MAG_DRDY) != 0)),
            "overflow": int(np.sum((flags & CAL_MAG_OVERFLOW) != 0)),
            "valid": int(np.sum((flags & FLAG_MAG_VALID) != 0)),
        },
        "encoder": {
            "left_last": int(encoder[-1, 0]),
            "right_last": int(encoder[-1, 1]),
            "left_changes": int(np.sum(np.diff(encoder[:, 0]) != 0)),
            "right_changes": int(np.sum(np.diff(encoder[:, 1]) != 0)),
        },
    }


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
    yaw_sigma_deg = gyro_sigma_deg[0]
    if data.shape[1] >= 55:
        yaw_sigma_deg = np.rad2deg(np.std(_detrend(
            gyro_rows[:, COL_TIME], gyro_rows[:, 48])))

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
        ("gyro yaw (deg/s)", SIGMA_GYRO_DEG, yaw_sigma_deg),
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
                              (KF_WHEEL_OMEGA, "wheel omega", 2)):
        row = _nis_table(data, bit, label, index)
        if row is None:
            continue
        nis_rows.append(row)
        print("%-14s %6d %10.3f %10.3f %10.3f %10.3f" %
              (label, row["n"], row["mean"], row["median"],
               row["p95"], row["frac_gt_3.84"]))
    if nis_rows:
        print("NIS ~ 1 is consistent; mean >> 1 means R too small or P too "
              "confident. The wheel v/omega rows share angle noise, so treat "
              "those two as approximate.")
    else:
        print("NIS unavailable: the stationary filter uses constant gains "
              "and does not propagate an innovation variance S.")

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


def _step_metrics(t, y, target, baseline, u, start, end,
                  voltage_limit=None, minimum_band=0.5):
    y_seg = y[start:end]
    t_seg = t[start:end]
    delta = target - baseline
    direction = 1.0 if delta >= 0 else -1.0
    if abs(delta) < 1e-9 or t_seg.size < 3:
        return None

    frac = (y_seg - baseline) / delta
    i10 = np.flatnonzero(frac >= 0.1)
    i90 = np.flatnonzero(frac >= 0.9)
    rise = float("nan")
    if i10.size and i90.size:
        rise = float(t_seg[i90[0]] - t_seg[i10[0]])

    overshoot = max(0.0, float(np.max(direction * (y_seg - target))) /
                    abs(delta))

    band = max(0.05 * abs(delta), minimum_band)
    outside = np.flatnonzero(np.abs(y_seg - target) > band)
    settle = float("nan")
    if outside.size == 0:
        settle = 0.0
    elif outside[-1] + 1 < y_seg.size:
        settle = float(t_seg[outside[-1] + 1] - t_seg[0])

    tail = max(1, int(0.3 * y_seg.size))
    ss_error = float(np.mean(y_seg[-tail:] - target))
    rms = float(np.sqrt(np.mean((y_seg - target) ** 2)))
    limit = CONTROL_VOLTAGE_LIMIT if voltage_limit is None else voltage_limit
    sat = float(np.mean(np.abs(u[start:end]) >= 0.99 * limit))
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


def _phase_window(t, phase, target):
    """Rows of one schedule phase, with the post-transition settle excluded.

    The settling phase is no longer than the exclusion window, so it is
    returned whole and flagged.
    """
    mask = phase == target
    if not mask.any():
        return mask, False
    trimmed = mask & (t >= t[mask][0] + SETTLE_EXCLUDE)
    if trimmed.sum() < 10:
        return mask, False
    return trimmed, True


def _sigma(t, x, mask):
    """Detrended 1-sigma of x over mask."""
    tt = t[mask]
    xx = x[mask]
    good = np.isfinite(tt) & np.isfinite(xx)
    if good.sum() < 3:
        return float("nan")
    return float(np.std(_detrend(tt[good], xx[good])))


def _fit_first_order_tau(t, y, ref, start, duration=0.5):
    """Fit dy/dt = (ref - y) / tau over the first part of a step."""
    window = (t >= start) & (t <= start + duration)
    tt = t[window]
    yy = y[window]
    rr = ref[window]
    if tt.size < 5:
        return float("nan")
    dt = np.diff(tt)
    dy = np.diff(yy)
    err = rr[:-1] - yy[:-1]
    span = float(np.nanmax(np.abs(err)))
    if not np.isfinite(span) or span <= 0.0:
        return float("nan")
    good = (dt > 0.0) & np.isfinite(err) & np.isfinite(dy)
    good &= np.abs(err) > 0.05 * span
    if good.sum() < 5:
        return float("nan")
    slope = float(np.dot(err[good], dy[good] / dt[good]) /
                  np.dot(err[good], err[good]))
    if slope <= 0.0:
        return float("nan")
    return 1.0 / slope


def _ratio_rms(y, ref, mask):
    """Mean ratio and RMS difference of two signals over a steady window."""
    good = mask & np.isfinite(y) & np.isfinite(ref)
    if good.sum() < 3:
        return float("nan"), float("nan")
    ref_mean = float(np.mean(ref[good]))
    ratio = (float(np.mean(y[good])) / ref_mean
             if abs(ref_mean) > 1e-9 else float("nan"))
    rms = float(np.sqrt(np.mean((y[good] - ref[good]) ** 2)))
    return ratio, rms


def analyze_upside(t, data, outdir=None, show=False):
    """Estimator and sensor performance during the wheels-off schedule.

    The robot is secured upside down, so wheel-derived velocity is not chassis
    ground truth. The report therefore separates what the schedule can measure
    (sensor noise, wheel-loop tracking, filter response, bias/gyro coupling)
    from what it cannot (absolute body velocity and heading).
    """
    flags = data[:, COL_FLAGS].astype(np.uint16)
    kf_mask = data[:, COL_KFMASK].astype(np.uint16)
    phase = data[:, COL_PHASE].astype(int)

    feedback = data[:, COL_FEEDBACK:COL_FEEDBACK + 2]
    gyro = data[:, COL_GYRO:COL_GYRO + 3]
    acc = data[:, COL_ACC:COL_ACC + 3]
    as5_angle = data[:, COL_AS5ANGLE:COL_AS5ANGLE + 2]
    as5_vel = data[:, COL_AS5VEL:COL_AS5VEL + 2]

    kf_v = data[:, COL_KFV]
    kf_om = data[:, COL_KFOMEGA]
    kf_theta = np.unwrap(data[:, COL_KFTHETA])
    kf_ba = data[:, COL_KFBA]

    gyro_yaw_deg = YAW_GYRO_SIGN * gyro[:, YAW_GYRO_AXIS]
    accel_forward = FORWARD_ACCEL_SIGN * acc[:, FORWARD_ACCEL_AXIS] * 9.80665
    enc_v = 0.5 * (feedback[:, 0] + feedback[:, 1]) * WHEEL_RADIUS / GEAR
    enc_om = ((feedback[:, 1] - feedback[:, 0]) /
              WHEEL_BASE * WHEEL_RADIUS / GEAR)

    wheel_rows = (kf_mask & (KF_WHEEL_V | KF_WHEEL_OMEGA)) != 0
    fallback = (kf_mask & KF_WHEEL_ENCODER) != 0
    as5_fresh = (flags & FLAG_AS5_FRESH) != 0
    mag_valid = int(np.sum((flags & FLAG_MAG_VALID) != 0))

    # Sensor health. A dead SPI bus returns a constant frame with valid parity,
    # so the angle and the derived velocity both stay at zero while the
    # quadrature encoders show motion.
    as5_ptp = np.ptp(as5_angle, axis=0)
    wheel_moved = bool(np.any(np.abs(feedback) > ENCODER_MOTION))
    as5_dead = bool(np.nanmax(as5_ptp) < 1e-6
                    and not np.any(np.abs(as5_vel) > 1e-9)
                    and wheel_moved)
    zero_feed = wheel_rows & ~fallback & (
        np.max(np.abs(feedback), axis=1) > ENCODER_MOTION)

    fallback_share = {}
    for target in PHASE_ORDER:
        rows = (phase == target) & wheel_rows
        fallback_share[target] = (float((rows & fallback).sum() / rows.sum())
                                  if rows.sum() else float("nan"))

    # Reconstruct the velocity the filter was fed. With a dead AS5X47 the
    # encoder fallback is the only signal available.
    if as5_dead or not as5_fresh.any():
        ref_v = enc_v
        ref_om = enc_om
        source = ("encoder feedback (AS5X47 dead)" if as5_dead
                  else "encoder feedback (no AS5X47 samples)")
    else:
        idx = np.flatnonzero(as5_fresh)
        ref_v = np.interp(np.arange(len(t)), idx,
                          0.5 * (as5_vel[idx, 0] + as5_vel[idx, 1]))
        ref_om = np.interp(np.arange(len(t)), idx,
                           (as5_vel[idx, 1] - as5_vel[idx, 0]) / WHEEL_BASE)
        source = "AS5X47 velocity"

    print("\n=== Sensor health ===")
    print("AS5X47 angle peak-to-peak: L %.6f rad, R %.6f rad"
          % (as5_ptp[0], as5_ptp[1]))
    if as5_dead:
        print("AS5X47: DEAD BUS (constant angle and zero velocity while the "
              "wheels moved).")
        print("         The filter's wheel measurement is the encoder "
              "fallback, not an independent sensor.")
    else:
        print("AS5X47: angle varies; wheel velocity is available.")
    for target in PHASE_ORDER:
        share = fallback_share[target]
        if np.isfinite(share):
            print("encoder fallback share, phase %2d %-8s: %5.1f%%"
                  % (target, PHASE_LABELS[target], 100.0 * share))
    print("zero-velocity wheel updates while a wheel exceeded %.0f rad/s: "
          "%d (%.1f%% of visible updates)"
          % (ENCODER_MOTION, int(zero_feed.sum()),
             100.0 * zero_feed.sum() / max(1, int(wheel_rows.sum()))))
    if mag_valid == 0:
        print("magnetometer: UNAVAILABLE (0 valid heading samples); it is "
              "diagnostic-only and is not fused by the estimator.")
    else:
        print("magnetometer: %d valid heading samples" % mag_valid)

    print("\n=== Sensor noise (1-sigma, %.1f s after each transition "
          "excluded) ===" % SETTLE_EXCLUDE)
    print("%-3s %-9s %5s %10s %12s %8s %8s  %s"
          % ("ph", "phase", "n", "gyro yaw", "accel fwd", "ratio g",
             "ratio a", "window"))
    noise = {}
    for target in PHASE_ORDER:
        rows, trimmed = _phase_window(t, phase, target)
        if not rows.any():
            continue
        gyro_sigma = _sigma(t, gyro_yaw_deg, rows)
        accel_sigma = _sigma(t, accel_forward, rows)
        noise[target] = {
            "n": int(rows.sum()),
            "gyro_yaw_deg": gyro_sigma,
            "accel_forward": accel_sigma,
            "gyro_ratio": (gyro_sigma / SIGMA_GYRO_DEG
                           if np.isfinite(gyro_sigma) else float("nan")),
            "accel_ratio": (accel_sigma / SIGMA_ACCEL
                            if np.isfinite(accel_sigma) else float("nan")),
            "trimmed": bool(trimmed),
        }
        print("%-3d %-9s %5d %9.3f  %11.4f %8.2f %8.2f  %s"
              % (target, PHASE_LABELS[target], rows.sum(), gyro_sigma,
                 accel_sigma, noise[target]["gyro_ratio"],
                 noise[target]["accel_ratio"],
                 "settled" if trimmed else "whole phase"))
    print("assumed: gyro yaw %.3f deg/s, accel forward %.4f m/s^2"
          % (SIGMA_GYRO_DEG, SIGMA_ACCEL))

    fwd = np.flatnonzero(phase == 11)
    turn = np.flatnonzero(phase == 12)
    settle = np.flatnonzero(phase == 2)
    fwd_start = float(t[fwd[0]]) if fwd.size else float("nan")
    turn_start = float(t[turn[0]]) if turn.size else float("nan")

    tau = _fit_first_order_tau(t, kf_v, ref_v, fwd_start)
    tau_design = -WHEEL_WINDOW_NOMINAL / np.log(1.0 - KF_WHEEL_V0)
    v_ratio, v_rms = _ratio_rms(
        kf_v, ref_v, (phase == 11) & (t >= fwd_start + SETTLE_EXCLUDE))
    om_ratio, om_rms = _ratio_rms(
        kf_om, ref_om, (phase == 12) & (t >= turn_start + SETTLE_EXCLUDE))

    print("\n=== Filter tracking (wheel reference: %s) ===" % source)
    print("v step at %.3f s: tau = %.1f ms (K_WHEEL_V[0]=%.2f predicts "
          "%.1f ms)" % (fwd_start, 1e3 * tau, KF_WHEEL_V0,
                        1e3 * tau_design))
    print("forward phase: KF/wheel v = %.3f, RMS diff = %.4f m/s"
          % (v_ratio, v_rms))
    print("turn phase:    KF/wheel omega = %.3f, RMS diff = %.4f rad/s"
          % (om_ratio, om_rms))
    if np.isfinite(tau) and np.isfinite(tau_design):
        print("verdict: %s"
              % ("matches the designed gain"
                 if 0.7 < tau / tau_design < 1.4
                 else "differs from the designed gain; check the wheel update"))

    print("\n=== Bias and heading behaviour ===")
    print("b_a: start %+.4f, end %+.4f, swing %.4f m/s^2"
          % (kf_ba[0], kf_ba[-1], np.nanmax(kf_ba) - np.nanmin(kf_ba)))
    print("reserved bias column: %.4f (former b_g; not estimated)"
          % data[0, COL_KFBG])
    print("theta: total change %+.4f rad (%+.1f deg) while the body did not "
          "rotate" % (kf_theta[-1] - kf_theta[0],
                      np.degrees(kf_theta[-1] - kf_theta[0])))
    if turn.size:
        turn_gyro = float(np.mean(gyro_yaw_deg[phase == 12]))
        turn_wheel = float(np.mean(ref_om[phase == 12]))
        print("turn phase: wheel omega %.3f rad/s, gyro yaw %.3f deg/s"
              % (turn_wheel, turn_gyro))
    if settle.size:
        print("settling phase: v %+.4f m/s, omega %+.4f rad/s"
              % (kf_v[settle[-1]], kf_om[settle[-1]]))

    result = {
        "sensor": {
            "as5_dead": as5_dead,
            "as5_angle_ptp": float(np.nanmax(as5_ptp)),
            "mag_valid": mag_valid,
            "fallback_share": fallback_share,
            "zero_velocity_updates": int(zero_feed.sum()),
        },
        "noise": noise,
        "tracking": {
            "reference": source,
            "tau": tau,
            "tau_design": float(tau_design),
            "v_ratio": v_ratio,
            "v_rms": v_rms,
            "omega_ratio": om_ratio,
            "omega_rms": om_rms,
        },
        "bias": {
            "ba_start": float(kf_ba[0]),
            "ba_end": float(kf_ba[-1]),
            "ba_swing": float(np.nanmax(kf_ba) - np.nanmin(kf_ba)),
            "theta_change": float(kf_theta[-1] - kf_theta[0]),
        },
    }

    if outdir is not None or show:
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
        for target in PHASE_ORDER:
            rows = np.flatnonzero(phase == target)
            if rows.size == 0:
                continue
            color = {11: "tab:blue", 12: "tab:orange"}.get(target, "tab:gray")
            for ax in axes:
                ax.axvspan(t[rows[0]], t[rows[-1]], alpha=0.07, color=color)
        axes[0].plot(t, ref_v, lw=0.9, alpha=0.8, label=source)
        axes[0].plot(t, kf_v, lw=1.2, label="KF v")
        axes[0].set_ylabel("velocity (m/s)")
        axes[0].legend(loc="best")
        axes[0].grid(True, alpha=0.3)
        axes[1].plot(t, ref_om, lw=0.9, alpha=0.8, label=source)
        axes[1].plot(t, kf_om, lw=1.2, label="KF omega")
        axes[1].set_ylabel("yaw rate (rad/s)")
        axes[1].legend(loc="best")
        axes[1].grid(True, alpha=0.3)
        axes[2].plot(t, kf_ba, lw=1.1, label="b_a (m/s^2)")
        axes[2].set_ylabel("accel bias estimate")
        axes[2].set_xlabel("time (s)")
        axes[2].legend(loc="best")
        axes[2].grid(True, alpha=0.3)
        fig.tight_layout()
        _finish_fig(fig, outdir, "cal_upside")

    return result


def analyze_yaw(t, data, outdir=None, show=False):
    from analyze_tuning import capture
    config = {"radius": WHEEL_RADIUS, "gear": GEAR, "base": WHEEL_BASE,
              "ppr": 48, "limit": CONTROL_VOLTAGE_LIMIT}
    result = capture(data, "yaw", config)
    print("\n=== Isolated yaw PI (calibrated gyro feedback, heading disabled) ===")
    for m in result["steps"]:
        print("phase %d target %+.4f rad/s rise %.3f s overshoot %.1f%% "
              "SS error %+.4f limited-reference RMS %.4f correction saturation %.1f%%" % (
            m["phase"], m["target"], m["rise"], 100*m["overshoot"],
            m["ss_error"], m["limited_reference_rms"], 100*m["saturation"]))
    print("Encoder angle %.2f deg; gyro angle %.2f deg" % (
        result["wheel_angle_deg"], result["gyro_angle_deg"]))
    if outdir is not None or show:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(11, 5))
        for col, label in ((45, "programmed"), (46, "limited reference"),
                           (47, "command"), (48, "gyro feedback")):
            ax.plot(t, data[:, col], label=label)
        ax.set(xlabel="time (s)", ylabel="yaw rate (rad/s)")
        ax.legend()
        ax.grid(True, alpha=.3)
        fig.tight_layout()
        _finish_fig(fig, outdir, "cal_yaw")
    return result


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
    # Historical captures used 3 V; use their recorded limit when present.
    import re
    global CONTROL_VOLTAGE_LIMIT
    match = re.search(r"% Wheel voltage limit\s+([\d.eE+-]+)", path.read_text())
    if match:
        CONTROL_VOLTAGE_LIMIT = float(match.group(1))
    data, mode = load_cal(path)
    t = data[:, COL_TIME]
    print("Log:  %s" % path)
    print("Mode: %s" % mode)
    print("Samples: %d over %.3f s" % (len(t), t[-1] - t[0]))
    status = capture_status(data)
    if status:
        dt = status["imu_dt_us"]
        print("IMU interval (us): median %.0f, min %.0f, max %.0f"
              % (dt["median"], dt["min"], dt["max"]))
        mag = status["mag"]
        print("mag: attempts %d, DRDY %d, overflow %d, valid %d"
              % (mag["attempt"], mag["drdy"], mag["overflow"], mag["valid"]))

    show = not args.no_plots and args.outdir is None
    outdir = args.outdir
    if mode == "mag":
        analyze_mag(t, data, outdir=outdir, show=show)
    elif mode == "noise":
        analyze_noise(t, data, outdir=outdir, show=show)
    elif mode == "track":
        analyze_track(t, data, outdir=outdir, show=show)
    elif mode == "upside":
        analyze_upside(t, data, outdir=outdir, show=show)
    elif mode == "yaw":
        analyze_yaw(t, data, outdir=outdir, show=show)
    else:
        raise SystemExit("Unknown calibration mode %r" % mode)
    return 0


if __name__ == "__main__":
    sys.exit(main())
