#!/usr/bin/env python3
"""Console tools for measured yaw calibration and Basebot PI tuning trials."""

import argparse
import re
from pathlib import Path

import numpy as np

from analyze_calibration import load_cal, _step_metrics
from analyze_stream import load_stream


def metadata(path):
    text = Path(path).read_text()
    def number(pattern, default):
        match = re.search(pattern, text)
        return float(match.group(1)) if match else default
    return {
        "gear": number(r", gear ([\d.eE+-]+)", 9.6),
        "radius": number(r", wheel radius ([\d.eE+-]+)", 0.0315),
        "base": number(r"m, wheel base ([\d.eE+-]+)", 0.14),
        "ppr": number(r"PPR (\d+)", 48),
        "limit": number(r"% Wheel voltage limit\s+([\d.eE+-]+)", 7),
    }


def fresh(timestamp):
    timestamp = np.asarray(timestamp)
    return (timestamp != 0) & np.r_[True, np.diff(timestamp) != 0]


def integrate(t, y):
    return float(np.sum(np.diff(t) * (y[:-1] + y[1:]) / 2))


def measurement_time(timestamp):
    # micros() wraps after about 71 minutes; signed deltas unwrap one capture.
    delta = (np.diff(timestamp) + 2**31) % 2**32 - 2**31
    return np.r_[0, np.cumsum(delta)] * 1e-6


def validate_time(t):
    if len(t) < 3 or not np.all(np.isfinite(t)) or np.any(np.diff(t) <= 0):
        raise ValueError("Capture needs at least three samples with increasing time.")


def rotation(data, angle_deg, rest_seconds=1.0, axis=0, sign=-1, scale=1):
    """One measured rotation per stream, with stationary start/end windows."""
    t = data[:, 0]
    validate_time(t)
    if not np.isfinite(angle_deg) or angle_deg == 0 or rest_seconds <= 0 or not np.isfinite(scale) or scale <= 0:
        raise ValueError("Use a nonzero measured angle and positive rest window.")
    valid = fresh(data[:, 19]) if data.shape[1] >= 21 else (
        data[:, 2].astype(int) & 2) != 0
    t = measurement_time(data[valid, 19]) if data.shape[1] >= 21 else t[valid]
    gyro = data[valid, 5:8]
    validate_time(t)
    if t[-1] - t[0] <= 2 * rest_seconds:
        raise ValueError("Capture is too short for the stationary end windows.")
    first = t <= t[0] + rest_seconds
    last = t >= t[-1] - rest_seconds
    if min(np.sum(first), np.sum(last)) < 3:
        raise ValueError("Need at least three fresh samples in each rest window.")
    offset = (np.mean(gyro[first], axis=0) + np.mean(gyro[last], axis=0)) / 2
    angles = np.array([integrate(t, gyro[:, k] - offset[k]) for k in range(3)])
    best = int(np.argmax(np.abs(angles)))
    if abs(angles[best]) < 1e-6:
        raise ValueError("No rotation detected.")
    fitted_sign = 1 if angles[best] * angle_deg > 0 else -1
    fitted_scale = abs(angle_deg / angles[best])
    current_angle = sign * scale * angles[axis]
    return {"angles_deg": angles, "offset_dps": offset, "axis": best,
            "sign": fitted_sign, "scale": fitted_scale,
            "current_angle_deg": current_angle,
            "error_fraction": abs(current_angle - angle_deg) / abs(angle_deg),
            "rest_drift_dps": np.mean(gyro[last], axis=0) - np.mean(gyro[first], axis=0)}


def encoder_position(data, config):
    counts = data[:, 43:45]
    # Captures preserve signed low 16 bits; unwrap differences before scaling.
    delta = (np.diff(counts, axis=0) + 32768) % 65536 - 32768
    distance = np.vstack([np.zeros(2), np.cumsum(delta, axis=0)])
    return distance * (2 * np.pi * config["radius"] / config["gear"] / config["ppr"])


def capture(data, mode, config):
    t = data[:, 0]
    validate_time(t)
    if data.shape[1] < 55:
        raise ValueError("Tuning capture needs the new 55-column firmware log.")
    position = encoder_position(data, config)
    gyro_valid = fresh(data[:, 51])
    wheel_valid = fresh(data[:, 52])
    phase = data[:, 40].astype(int)
    flags = data[:, 54].astype(int)
    result = {"distance_m": float(np.mean(position[-1])),
              "wheel_angle_deg": float(np.rad2deg((position[-1, 1] - position[-1, 0]) / config["base"])),
              "gyro_angle_deg": float(np.rad2deg(integrate(
                  measurement_time(data[gyro_valid, 51]), data[gyro_valid, 48]))) if np.sum(gyro_valid) >= 3 else float("nan"),
              "steps": [], "noise": []}
    # Segment by transitions, including consecutive zero phases.
    starts = np.r_[0, np.flatnonzero(np.diff(phase)) + 1]
    for i, start in enumerate(starts):
        end = starts[i + 1] if i + 1 < len(starts) else len(t)
        if end - start < 5:
            continue
        # Use the second half of a plateau, at least 0.5 s after its edge;
        # yaw acceleration ramps can last longer than half a second.
        settled = np.zeros(len(t), dtype=bool)
        trim = max(0.5, (t[end - 1] - t[start]) / 2)
        settled[start:end] = t[start:end] >= t[start] + trim
        for source, bit in (("AS5X47", False), ("encoder", True)):
            mask = settled & wheel_valid & (((flags & 16) != 0) == bit)
            if np.sum(mask) >= 10:
                residual_omega = data[mask, 50] - data[mask, 48]
                result["noise"].append({"phase": int(phase[start]), "source": source,
                    "n": int(np.sum(mask)),
                    "yaw_bias": float(np.mean(residual_omega)),
                    "yaw_residual_sigma": float(np.std(residual_omega, ddof=1)),
                    "wheel_v_variation": float(np.std(data[mask, 49], ddof=1))})
        if mode == "noise":
            continue
        signals = [("yaw", data[:, 48], data[:, 46], data[:, 47])] if mode == "yaw" else [
            ("left", data[:, 6], data[:, 4], data[:, 8]),
            ("right", data[:, 7], data[:, 5], data[:, 9])]
        for name, y, reference, u in signals:
            target = float(np.median(reference[start:end]))
            baseline = float(np.median(y[max(0, start - 10):start])) if start else float(y[0])
            if abs(target - baseline) < (0.01 if mode == "yaw" else 2):
                continue
            m = _step_metrics(t, y, target, baseline, u, start, end,
                              voltage_limit=config["limit"],
                              minimum_band=0.01 if mode == "yaw" else 0.5)
            m.update({"loop": name, "phase": int(phase[start]), "target": target,
                      "command_sigma": float(np.std(u[start:end])),
                      "limited_reference_rms": float(np.sqrt(np.mean((y[start:end] - reference[start:end])**2)))})
            if mode == "yaw":
                m["saturation"] = float(np.mean((flags[start:end] & 256) != 0))
                m["wheel_saturation"] = float(np.mean((flags[start:end] & 96) != 0))
            else:
                wheel = 0 if name == "left" else 1
                duration = t[end] - t[start] if end < len(t) else t[-1] - t[start]
                stop = min(end, len(t) - 1)
                m["encoder_distance_m"] = float(position[stop, wheel] - position[start, wheel])
                m["reference_distance_m"] = target * config["radius"] / config["gear"] * duration
            result["steps"].append(m)
    if mode == "noise":
        accel_valid = fresh(data[:, 53])
        if min(np.sum(gyro_valid), np.sum(accel_valid)) < 3:
            raise ValueError("Noise capture needs fresh gyro and accelerometer samples.")
        result["stationary"] = {
            "sigma_gyro_dps": float(np.std(np.rad2deg(data[gyro_valid, 48]), ddof=1)),
            "sigma_accel_mps2": float(np.std(data[accel_valid, 17] * 9.80665, ddof=1))}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    rotate = sub.add_parser("rotation", help="One measured signed rotation, with 1 s rest at both ends")
    rotate.add_argument("log", type=Path)
    rotate.add_argument("--angle-deg", type=float, required=True, help="Measured angle; positive CCW")
    rotate.add_argument("--rest-seconds", type=float, default=1)
    rotate.add_argument("--axis", type=int, choices=range(3), default=0)
    rotate.add_argument("--sign", type=int, choices=(-1, 1), default=-1)
    rotate.add_argument("--scale", type=float, default=1)
    trial = sub.add_parser("capture", help="Track, yaw, or stationary-noise tuning capture")
    trial.add_argument("log", type=Path)
    args = parser.parse_args()
    try:
        if args.command == "rotation":
            r = rotation(load_stream(args.log), args.angle_deg, args.rest_seconds,
                         args.axis, args.sign, args.scale)
            print("Integrated reported gyro axes (deg):", r["angles_deg"])
            print("Stationary residual offsets (deg/s):", r["offset_dps"])
            print("Start/end bias change (deg/s):", r["rest_drift_dps"])
            print("Current mapping: %.2f deg, error %.1f%%" % (r["current_angle_deg"], 100*r["error_fraction"]))
            print("Measured candidate: yawAxis=%d yawSign=%+d yawScale=%.6f" % (r["axis"], r["sign"], r["scale"]))
            print("Repeat both directions before changing tuning.h; verify cal imu range readback first.")
        else:
            data, mode = load_cal(args.log)
            if mode not in ("track", "yaw", "noise"):
                raise ValueError("Use cal track, cal yaw, or cal noise.")
            r = capture(data, mode, metadata(args.log))
            print("Encoder distance %.4f m, encoder angle %.2f deg, gyro angle %.2f deg" % (
                r["distance_m"], r["wheel_angle_deg"], r["gyro_angle_deg"]))
            for m in r["steps"]:
                print("phase %d %-5s target %+.4f rise %.3fs overshoot %.1f%% SS error %+.4f "
                      "reference RMS %.4f command SD %.4f saturation %.1f%%" % (
                    m["phase"], m["loop"], m["target"], m["rise"], 100*m["overshoot"],
                    m["ss_error"], m["limited_reference_rms"], m["command_sigma"], 100*m["saturation"]))
                if "encoder_distance_m" in m:
                    print("  count distance %.4f m; reference distance %.4f m" % (
                        m["encoder_distance_m"], m["reference_distance_m"]))
                if "wheel_saturation" in m:
                    print("  wheel voltage saturation %.1f%%" % (100*m["wheel_saturation"]))
            if "stationary" in r:
                print("Stationary DARE noise inputs:", r["stationary"])
            for n in r["noise"]:
                print("phase %d %s n=%d wheel-minus-gyro bias %+.4f rad/s, residual SD %.4f rad/s, "
                      "wheel-v variation %.4f m/s" % (n["phase"], n["source"], n["n"],
                    n["yaw_bias"], n["yaw_residual_sigma"], n["wheel_v_variation"]))
            print("Moving variation includes real motion; do not treat it all as sensor R. "
                  "Decimated captures cannot replay every estimator update.")
    except ValueError as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
