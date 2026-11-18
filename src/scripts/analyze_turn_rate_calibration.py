#!/usr/bin/env python3
"""Analyze the latest start-button Turn-rate calibration sequence log."""

import argparse
from pathlib import Path
import re

import numpy as np

HEADER = "% CAL turn-rate-calibration"


def load_capture(path):
    """Select the last buffered calibration block, including an aborted run."""
    lines = Path(path).read_text().splitlines()
    starts = [i for i, line in enumerate(lines) if line.strip() == HEADER]
    for start in reversed(starts):
        stop = next((i for i in range(start + 1, len(lines))
                     if lines[i].startswith(("% CAL ", "% Basebot log"))), len(lines))
        block = lines[start:stop]
        rows = []
        for line in block:
            if not line or line.startswith("%"):
                continue
            try:
                row = [float(value) for value in line.split()]
            except ValueError:
                continue
            if len(row) == 62:
                rows.append(row)
        if rows:
            text = "\n".join(block)
            phases = [list(map(float, line.split()[2:])) for line in block
                      if line.startswith("% turn-phase ")]
            return np.asarray(rows), np.asarray(phases).reshape(-1, 10), text
    raise ValueError("No 62-column turn-rate calibration capture in %s" % path)


def latest_capture(directory):
    for path in sorted(Path(directory).glob("*.log"),
                       key=lambda item: item.stat().st_mtime, reverse=True):
        try:
            load_capture(path)
        except ValueError:
            continue
        return path
    raise ValueError("No turn-rate calibration log in %s" % directory)


def vector(text, prefix, default):
    match = re.search(r"(?m)^" + re.escape(prefix) + r"\s+([^\n]+)", text)
    return np.fromstring(match[1], sep=" ") if match else np.asarray(default)


def analyze(data, phases, text):
    mapping = vector(text, "% Yaw mapping axis,sign,scale", [0, -1, 1])
    axis, sign, scale = int(mapping[0]), mapping[1], mapping[2]
    offsets = vector(text, "% Gyro offsets (library deg/s)", [0, 0, 0])
    calibrated = (data[:, 2].astype(int) & 2048) != 0
    conversion_error = float("nan")
    if np.any(calibrated):
        expected = -data[calibrated, 55:58] * (1000 / 32768) - offsets
        conversion_error = float(np.max(np.abs(expected - data[calibrated, 12:15])))
    turns = phases[phases[:, 1] != 0]
    result = {"mapping": mapping, "conversion_error_dps": conversion_error,
              "gravity_g": vector(text, "% Stationary gravity (library g)", [np.nan]*3),
              "turns": [], "fits": []}
    if not len(turns):
        return result
    duration = turns[:, 3] - turns[:, 2]
    wheel_rate = turns[:, 7] / duration
    gyro_rate = turns[:, 4:7] / duration[:, None]
    result["dominant_axis"] = int(np.argmax(np.sum(np.abs(turns[:, 4:7]), axis=0)))
    for row, period in zip(turns, duration):
        gyro_angle = sign * scale * row[4 + axis]
        result["turns"].append({"phase": int(row[0]), "reference": row[1],
            "duration": period, "gyro_deg": np.rad2deg(gyro_angle),
            "wheel_deg": np.rad2deg(row[7]),
            "wheel_over_gyro": row[7] / gyro_angle if abs(gyro_angle) > 1e-6 else np.nan,
            "gyro_hz": row[8] / period, "saturation": row[9]})
    # Fit signed rate across both directions to distinguish scale from offset.
    if len(turns) >= 2 and np.ptp(wheel_rate) > 1e-6:
        for k in range(3):
            slope, bias = np.polyfit(wheel_rate, gyro_rate[:, k], 1)
            result["fits"].append({"axis": k, "slope": slope, "bias": bias})
        slope, bias = np.polyfit(wheel_rate, sign * scale * gyro_rate[:, axis], 1)
        result["mapped_slope"] = slope
        result["mapped_bias"] = bias
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", nargs="?", type=Path)
    args = parser.parse_args()
    try:
        path = args.log or latest_capture(Path(__file__).resolve().parents[1] / "logs")
        data, phases, text = load_capture(path)
        result = analyze(data, phases, text)
    except ValueError as exc:
        parser.error(str(exc))
    print("Turn-rate calibration sequence:", path)
    status = re.search(r"(?m)^% Result: (.+)", text)
    print("Result:", status[1] if status else "unknown")
    print("Stationary gravity (library g):", result["gravity_g"])
    print("Hardware count/library conversion max error: %.6g deg/s" % result["conversion_error_dps"])
    for row in result["turns"]:
        print("phase %d ref %+.2f gyro %+.2f deg wheel %+.2f deg ratio %.3f gyro %.1f Hz saturation %.1f%%" % (
            row["phase"], row["reference"], row["gyro_deg"], row["wheel_deg"],
            row["wheel_over_gyro"], row["gyro_hz"], row["saturation"]*100))
    if "dominant_axis" in result:
        print("Dominant reported gyro axis:", "XYZ"[result["dominant_axis"]])
    for fit in result["fits"]:
        print("gyro %s = %.5f * wheel yaw + %+.5f rad/s" % (
            "XYZ"[fit["axis"]], fit["slope"], fit["bias"]))
    if "mapped_slope" in result:
        print("Mapped gyro = %.5f * wheel yaw + %+.5f rad/s" % (
            result["mapped_slope"], result["mapped_bias"]))
    print("Floor runs compare body gyro with wheel motion. Upside-down wheel yaw is not body yaw.")


if __name__ == "__main__":
    main()
