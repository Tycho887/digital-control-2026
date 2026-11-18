#!/usr/bin/env python3
"""Analyze a `cal stream` capture for sensor noise and Allan deviation.

The firmware streams raw IMU, magnetometer, AS5X47 and encoder samples at
50-500 Hz while the robot is stationary. The columns are documented by the
`% 1 ...` header printed by `streamHeader()` in `basebot_6.ino`.

Examples:
    python3 src/scripts/analyze_stream.py
    python3 src/scripts/analyze_stream.py src/logs/device-monitor-....log
    python3 src/scripts/analyze_stream.py --outdir /tmp/stream-plots
"""

import argparse
import re
from pathlib import Path

import numpy as np

GRAVITY = 9.80665

COL_TIME = 0
COL_DT = 1
COL_FLAGS = 2
COL_ST1 = 3
COL_ST2 = 4
COL_GYRO = 5
COL_ACC = 8
COL_MAG = 11
COL_AS5 = 14
COL_ENCODER = 16
COL_TEMP = 18
N_COLUMNS = 19

FLAG_ACCEL_FRESH = 1 << 0
FLAG_GYRO_FRESH = 1 << 1
FLAG_MAG_FRESH = 1 << 2
FLAG_MAG_VALID = 1 << 3
FLAG_MAG_AVAILABLE = 1 << 4
FLAG_AS5_VALID = 1 << 5
FLAG_AS5_FRESH = 1 << 6
CAL_MAG_ATTEMPT = 1 << 8
CAL_MAG_DRDY = 1 << 9
CAL_MAG_OVERFLOW = 1 << 10


def latest_stream_log(directory):
    logs = sorted(directory.glob("*.log"), key=lambda p: p.stat().st_mtime,
                  reverse=True)
    for path in logs:
        try:
            head = path.open(errors="replace").read(16384)
        except OSError:
            continue
        if "% CAL stream" in head:
            return path
    raise SystemExit("No '%% CAL stream' capture found in %s" % directory)


def load_stream(path):
    lines = Path(path).read_text(errors="replace").splitlines()
    start = None
    for i, line in enumerate(lines):
        if line.startswith("% CAL stream"):
            start = i
            break
    if start is None:
        raise SystemExit("%s has no '%% CAL stream' header." % path)
    rows = []
    data_started = False
    for line in lines[start + 1:]:
        if line.startswith("%"):
            if rows and (line.startswith("% CAL ")
                         or line.startswith("% Basebot log")
                         or line.startswith("% log format")):
                break
            if line.startswith("% 1 "):
                data_started = True
            continue
        if not line.strip() or not data_started:
            continue
        parts = line.split()
        if len(parts) < N_COLUMNS:
            break
        try:
            float(parts[0])
        except ValueError:
            break
        rows.append(parts)
    if not rows:
        raise SystemExit("%s contains no stream rows." % path)
    return np.array(rows, dtype=float)


def _sigma(t, x):
    x = np.asarray(x, dtype=float)
    if x.size < 3:
        return float("nan")
    c = np.polyfit(t - t[0], x, 1)
    return float(np.std(x - np.polyval(c, t - t[0])))


def allan_deviation(t, x, max_points=60):
    """Overlapping Allan deviation of a rate signal.

    Returns (tau, adev) with tau in seconds. The white-noise region has
    adev ~ q/sqrt(tau), so q is the rate noise density.
    """
    x = np.asarray(x, dtype=float)
    x = x - np.mean(x)
    dt = float(np.median(np.diff(t)))
    theta = np.cumsum(x) * dt
    n = x.size
    ms = []
    m = 1
    while m <= n // 9 and len(ms) < max_points:
        ms.append(m)
        m *= 2
    if not ms:
        return np.array([]), np.array([])
    taus = np.array(ms, dtype=float) * dt
    adev = np.empty(len(ms))
    for i, m in enumerate(ms):
        d = theta[2 * m:] - 2.0 * theta[m:-m] + theta[:-2 * m]
        adev[i] = np.sqrt(np.mean(d * d) / (2.0 * (m * dt) ** 2))
    return taus, adev


def white_noise_fit(taus, adev, points=4):
    """Fit adev = q/sqrt(tau) on the shortest taus."""
    good = np.isfinite(taus) & np.isfinite(adev) & (adev > 0)
    taus = taus[good]
    adev = adev[good]
    k = min(points, taus.size)
    if k < 2:
        return float("nan"), float("nan")
    slope = float(np.polyfit(np.log10(taus[:k]), np.log10(adev[:k]), 1)[0])
    q = float(np.median(adev[:k] * np.sqrt(taus[:k])))
    return q, slope


def analyze_stream(t, data, outdir=None, show=False,
                   yaw_axis=0, yaw_sign=-1, yaw_scale=1):
    flags = data[:, COL_FLAGS].astype(np.uint16)
    gyro = data[:, COL_GYRO:COL_GYRO + 3]
    acc = data[:, COL_ACC:COL_ACC + 3]
    mag = data[:, COL_MAG:COL_MAG + 3]

    gyro_yaw = yaw_sign * yaw_scale * gyro[:, yaw_axis] * np.pi / 180.0
    accel_forward = -acc[:, 2] * GRAVITY         # m/s^2
    dt_us = data[:, COL_DT]
    rate = 1 / float(np.median(np.diff(t)))
    if data.shape[1] >= 21:
        gyro_valid = (data[:, 19] != 0) & np.r_[True, np.diff(data[:, 19]) != 0]
        accel_valid = (data[:, 20] != 0) & np.r_[True, np.diff(data[:, 20]) != 0]
    else:
        gyro_valid = (flags & FLAG_GYRO_FRESH) != 0
        accel_valid = (flags & FLAG_ACCEL_FRESH) != 0

    print("\n=== Stream ===")
    print("samples %d over %.3f s, median rate %.1f Hz"
          % (len(t), t[-1] - t[0], rate))
    print("IMU interval us: median %.0f, min %.0f, max %.0f"
          % (np.median(dt_us), np.min(dt_us), np.max(dt_us)))
    print("temperature C: %s"
          % ("%.2f" % np.median(data[:, COL_TEMP])
             if np.isfinite(data[:, COL_TEMP]).any() else "n/a"))

    print("\n=== Flags ===")
    for name, bit in (("accel fresh", FLAG_ACCEL_FRESH),
                      ("gyro fresh", FLAG_GYRO_FRESH),
                      ("mag fresh", FLAG_MAG_FRESH),
                      ("mag valid", FLAG_MAG_VALID),
                      ("mag available", FLAG_MAG_AVAILABLE),
                      ("AS5 valid", FLAG_AS5_VALID),
                      ("mag attempt", CAL_MAG_ATTEMPT),
                      ("mag DRDY", CAL_MAG_DRDY),
                      ("mag overflow", CAL_MAG_OVERFLOW)):
        print("  %-14s %6d / %d" % (name, int(np.sum(
            (flags & bit) != 0)), len(flags)))
    print("  ST1 unique: %s" % np.unique(data[:, COL_ST1].astype(int))[:8])
    print("  ST2 unique: %s" % np.unique(data[:, COL_ST2].astype(int))[:8])

    print("\n=== Sensor noise (1-sigma) ===")
    gyro_sigma = [_sigma(t[gyro_valid], gyro[gyro_valid, k]) for k in range(3)]
    yaw_sigma = abs(yaw_scale) * gyro_sigma[yaw_axis]
    acc_sigma = [_sigma(t[accel_valid], acc[accel_valid, k]) * GRAVITY for k in range(3)]
    print("gyro deg/s: x=%.4f y=%.4f z=%.4f"
          % tuple(gyro_sigma))
    print("accel m/s^2: x=%.4f y=%.4f z=%.4f"
          % tuple(acc_sigma))
    print("gyro yaw %.4f deg/s (%.3e rad/s)"
          % (abs(yaw_sigma), abs(yaw_sigma) * np.pi / 180.0))
    print("accel forward %.4f m/s^2" % abs(acc_sigma[2]))

    mag_sigma = [_sigma(t, mag[:, k]) for k in range(3)]
    print("mag counts: x=%.3f y=%.3f z=%.3f" % tuple(mag_sigma))
    mag_valid = ((flags & FLAG_MAG_VALID) != 0) & ((flags & FLAG_MAG_FRESH) != 0)
    heading_sigma = float("nan")
    if mag_valid.sum() > 10:
        heading = np.unwrap(np.arctan2(mag[mag_valid, 1],
                                       -mag[mag_valid, 2]))
        tt = t[mag_valid]
        heading_sigma = _sigma(tt, heading)
        print("mag heading %.4f rad (%d samples)" % (heading_sigma,
                                                     mag_valid.sum()))
    else:
        print("mag heading: not available")

    print("\n=== Allan deviation ===")
    allan = {}
    for name, signal, valid, unit in (("gyro yaw", gyro_yaw, gyro_valid, "rad/sqrt(s)"),
                                     ("accel forward", accel_forward, accel_valid, "m/s^1.5")):
        taus, adev = allan_deviation(t[valid], signal[valid]) if np.sum(valid) >= 10 else (np.array([]), np.array([]))
        q, slope = white_noise_fit(taus, adev)
        adev_min = float(np.min(adev)) if adev.size else float("nan")
        tau_min = float(taus[int(np.argmin(adev))]) if adev.size else float("nan")
        print("%-14s q=%.4e %-11s slope=%+.2f  ADEV min=%.4e at tau=%.3f s"
              % (name, q, unit, slope, adev_min, tau_min))
        allan[name] = {
            "tau": taus,
            "adev": adev,
            "q": q,
            "unit": unit,
            "slope": slope,
            "min": adev_min,
            "tau_min": tau_min,
        }

    if outdir is not None or show:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(9, 6))
        for name, style in (("gyro yaw", "-o"), ("accel forward", "-s")):
            a = allan[name]
            if a["tau"].size:
                ax.loglog(a["tau"], a["adev"], style, ms=3, lw=1.0,
                          label=name)
        tau = allan["gyro yaw"]["tau"]
        if tau.size:
            ref = allan["gyro yaw"]["adev"][0] * np.sqrt(tau[0] / tau)
            ax.loglog(tau, ref, "k:", lw=1.0, label="slope -1/2")
        ax.set_xlabel("tau (s)")
        ax.set_ylabel("Allan deviation")
        ax.set_title("cal stream Allan deviation")
        ax.grid(True, which="both", alpha=0.3)
        ax.legend(loc="best")
        fig.tight_layout()
        _finish_fig(fig, outdir, "stream_allan")

    return {
        "rate": rate,
        "duration": float(t[-1] - t[0]),
        "sigma": {
            "gyro_yaw_deg": abs(yaw_sigma),
            "gyro_yaw_rad": abs(yaw_sigma) * np.pi / 180.0,
            "accel_forward": abs(acc_sigma[2]),
            "mag_x": mag_sigma[0],
            "mag_heading": heading_sigma,
        },
        "allan": allan,
        "mag_valid": int(mag_valid.sum()),
        "mag_attempts": int(np.sum((flags & CAL_MAG_ATTEMPT) != 0)),
        "mag_drdy": int(np.sum((flags & CAL_MAG_DRDY) != 0)),
        "mag_overflow": int(np.sum((flags & CAL_MAG_OVERFLOW) != 0)),
    }


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
        help="stream log (default: newest with a CAL stream header in %s)"
             % default_logs,
    )
    parser.add_argument(
        "--outdir", type=Path,
        help="save the Allan plot as a PNG in this directory",
    )
    parser.add_argument(
        "--no-plots", action="store_true",
        help="print the summary only",
    )
    args = parser.parse_args()

    path = args.log or latest_stream_log(default_logs)
    data = load_stream(path)
    t = data[:, COL_TIME]
    print("Log:  %s" % path)
    show = not args.no_plots and args.outdir is None
    mapping = re.search(r"% Yaw mapping axis,sign,scale\s+(\d+)\s+([\d.eE+-]+)\s+([\d.eE+-]+)", path.read_text())
    options = {}
    if mapping:
        options = {"yaw_axis": int(mapping[1]), "yaw_sign": float(mapping[2]),
                   "yaw_scale": float(mapping[3])}
    analyze_stream(t, data, outdir=args.outdir, show=show, **options)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
