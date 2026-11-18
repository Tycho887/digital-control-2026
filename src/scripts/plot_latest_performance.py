#!/usr/bin/env python3
"""Plot the newest format-4 Basebot log for controller tuning.

The script selects the newest usable log in src/logs (or the path given on
the command line), parses the format-4 rows, and writes three figures next to
the log:

    <log>_tracking.png    speed, yaw-rate, heading, distance tracking
    <log>_control.png     wheel loops, voltage/PWM, yaw/heading PID outputs
    <log>_estimator.png   KF state, accel bias, innovations, status summary

Column numbers are the documented format-4 columns (1-based in the firmware
header). The raw IMU (58–63), total angle / yaw PI (64–65), and wheel magnetic
diagnostics (66–71) are optional. Earlier format-4 logs still plot.

Usage:
    .venv/bin/python src/scripts/plot_latest_performance.py
    .venv/bin/python src/scripts/plot_latest_performance.py path/to/log.log
"""

import argparse
import re
import warnings
from collections import Counter
from pathlib import Path

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

# 0-based column indices from the format-4 header.
C_TIME = 0
C_STATE = 1
C_VREF = 2
C_MOTOR_V = 3          # 3-4
C_ENCODER = 5          # forward-positive quadrature counts, L/R
C_MOTOR_VEL = 7        # 7-8
C_X, C_Y, C_REL_HEADING = 10, 11, 12  # position (m), heading relative to start (rad)
C_DIST = 13
C_PWM = 19             # 19-20
C_CONTROL = 24         # 24-25
C_SETPOINT = 29        # 29-30
C_YAW_REF = 31
C_KF = 32              # 32-36
C_YAW_ERR = 38
C_V_CMD = 39
C_YAW_CMD = 40
C_FEEDBACK = 41        # 41-42
C_HEAD_REF = 45
C_HEAD_ERR = 46
C_OMEGA_REF = 47
C_HEAD_OUT = 48
C_INNOV = 51           # 51-54
C_KF_MASK = 55
C_FLAGS = 56
C_GYRO = 57            # 57-59, optional
C_ACC = 60             # 60-62, optional
C_TOTAL_ANGLE = 63     # optional
C_YAW_PID = 64         # optional
C_MAG, C_ANGLECOM, C_DIAAGC = 65, 67, 69  # optional L/R pairs, AS5047P/AS5147

MIN_COLUMNS = 57

PHASE_COLORS = {10: "#d9d9d9", 11: "#cfe2f3", 15: "#e7e2ed", 12: "#fce5cd", 13: "#f6eee4",
                14: "#d9ead3", 16: "#e7e2ed", 17: "#fce5cd", 2: "#efefef"}
PHASE_LABELS = {10: "rest", 11: "forward", 15: "pre-turn stop", 12: "turn", 13: "turn settle",
                14: "return", 16: "return stop", 17: "final turn", 2: "final settle"}


def parse_header(text):
    """Return trajectory/geometry metadata from the log header."""
    meta = {
        "gear": 9.6,
        "radius": 0.0315,
        "base": 0.14,
        "initial_rest": 0.0,
        "forward_duration": 0.0,
        "turn_duration": 0.0,
        "final_rest": 0.0,
        "forward_speed": 0.0,
        "turn_rate": 0.0,
    }

    match = re.search(
        r"% Sample time \d+ us, log divider \d+, gear ([\d.eE+-]+), "
        r"wheel radius ([\d.eE+-]+) m, wheel base ([\d.eE+-]+) m",
        text,
    )
    if match:
        meta["gear"] = float(match.group(1))
        meta["radius"] = float(match.group(2))
        meta["base"] = float(match.group(3))
    match = re.search(r"wheel base [\d.eE+-]+ m, PPR (\d+)", text)
    meta["ppr"] = int(match.group(1)) if match else None

    match = re.search(
        r"% Trajectory baseline,forward duration,turn duration,settling "
        r"\(s\)\s+([^\n]+)",
        text,
    )
    if match:
        values = [float(v) for v in match.group(1).split()]
        if len(values) >= 4:
            (meta["initial_rest"], meta["forward_duration"],
             meta["turn_duration"], meta["final_rest"]) = values[:4]

    match = re.search(
        r"% Trajectory forward speed \(m/s\), turn rate \(rad/s\)\s+([^\n]+)",
        text,
    )
    if match:
        values = [float(v) for v in match.group(1).split()]
        if len(values) >= 2:
            meta["forward_speed"], meta["turn_rate"] = values[:2]

    match = re.search(r"% Trajectory post-turn settle,return duration \(s\)\s+(\S+)\s+(\S+)", text)
    return_duration = float(match.group(2)) if match else 0.0
    meta["target_distance"] = meta["forward_speed"] * (meta["forward_duration"] + return_duration)
    meta["turn_angles"] = [meta["turn_rate"] * meta["turn_duration"]]
    match = re.search(r"% Final turn duration \(s\), mean rate \(rad/s\), pre-turn stop \(s\)\s+(\S+)\s+(\S+)", text)
    if match:
        meta["turn_angles"].append(float(match.group(1)) * float(match.group(2)))
    return meta


def load_log(path):
    """Load the last contiguous format-4 block from a monitor log."""
    text = path.read_text(errors="replace")
    if "% log format 4" not in text:
        raise ValueError("not a format-4 log")
    text = text.rsplit("% log format 4", 1)[1]

    rows = []
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("%"):
            continue
        parts = line.split()
        if len(parts) < MIN_COLUMNS:
            continue
        try:
            rows.append([float(value) for value in parts])
        except ValueError:
            continue
    if not rows:
        raise ValueError("no format-4 data rows")

    widths = Counter(len(row) for row in rows)
    width = widths.most_common(1)[0][0]
    rows = [row for row in rows if len(row) == width]
    data = np.asarray(rows, dtype=float)

    # A monitor log can contain several captures. Keep the last one.
    reset = np.flatnonzero(np.diff(data[:, C_TIME]) < -0.01)
    if reset.size:
        data = data[reset[-1] + 1:]
    if data.shape[0] < 10:
        raise ValueError("last capture is too short")
    return data, parse_header(text)


def latest_log(directory):
    for path in sorted(directory.glob("*.log"),
                       key=lambda item: item.stat().st_mtime, reverse=True):
        try:
            data, meta = load_log(path)
        except (OSError, ValueError):
            continue
        return path, data, meta
    raise SystemExit(f"No usable format-4 log found in {directory}")


def trapezoid(rate, t):
    return np.concatenate(
        [[0.0], np.cumsum(0.5 * (rate[1:] + rate[:-1]) * np.diff(t))])


def unwrap_from_start(angle):
    unwrapped = np.unwrap(angle)
    return unwrapped - unwrapped[0]


def shade_phases(ax, t, state):
    start = 0
    for index in range(1, len(t) + 1):
        if index == len(t) or state[index] != state[start]:
            color = PHASE_COLORS.get(int(state[start]), "#ffffff")
            ax.axvspan(t[start], t[min(index, len(t) - 1)],
                       color=color, alpha=0.45, lw=0, zorder=0)
            start = index


def phase_handles(state):
    return [Patch(facecolor=PHASE_COLORS[key], alpha=0.6,
                  label=PHASE_LABELS[key])
            for key in PHASE_LABELS if key in state]


def finish_axis(ax, ylabel, title, legend=True):
    ax.set_ylabel(ylabel)
    ax.set_title(title, loc="left")
    ax.grid(True, alpha=0.3)
    if legend:
        ax.legend(loc="best", fontsize=7, ncol=2)


def plot_tracking(path, data, meta, out_dir):
    t = data[:, C_TIME]
    state = data[:, C_STATE].astype(int)
    v_ref = data[:, C_VREF]
    v_cmd = data[:, C_V_CMD]
    kf = data[:, C_KF:C_KF + 5]
    heading_ref = data[:, C_HEAD_REF]
    yaw_ref = data[:, C_YAW_REF]
    yaw_cmd = data[:, C_YAW_CMD]
    yaw_err = data[:, C_YAW_ERR]
    dist = data[:, C_DIST]
    motor_vel = data[:, C_MOTOR_VEL:C_MOTOR_VEL + 2]

    v_enc = ((motor_vel[:, 0] + motor_vel[:, 1]) / 2.0) / meta["gear"] * meta["radius"]
    yaw_enc_rate = ((motor_vel[:, 1] - motor_vel[:, 0]) / meta["gear"]
                    * meta["radius"] / meta["base"])
    yaw_enc = trapezoid(yaw_enc_rate, t)

    kf_heading = unwrap_from_start(kf[:, 2])
    ref_heading = unwrap_from_start(heading_ref)
    if data.shape[1] > C_TOTAL_ANGLE:
        total_angle = data[:, C_TOTAL_ANGLE]
    else:
        total_angle = kf_heading
    heading_deg = np.rad2deg(total_angle)
    ref_heading_deg = np.rad2deg(ref_heading)
    enc_heading_deg = np.rad2deg(yaw_enc)
    target_angle = sum(meta["turn_angles"])
    target_distance = meta["target_distance"]
    ref_distance = trapezoid(v_ref, t)

    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True,
                             constrained_layout=True)
    fig.suptitle(f"Tracking — {path.name}", fontsize=11)

    for ax in axes.flat:
        shade_phases(ax, t, state)

    ax = axes[0, 0]
    ax.plot(t, v_ref, "k--", lw=1.2, label="v reference")
    ax.plot(t, v_cmd, "k:", lw=1.2, label="v command")
    ax.plot(t, kf[:, 0], lw=1.2, label="KF v")
    ax.plot(t, v_enc, lw=0.8, alpha=0.55, label="encoder v")
    finish_axis(ax, "m/s", "Forward speed")
    ax.legend(handles=phase_handles(state) + ax.get_legend_handles_labels()[0],
              loc="best", fontsize=7, ncol=2)

    ax = axes[0, 1]
    ax.plot(t, v_ref - kf[:, 0], label="v reference - KF v")
    ax.plot(t, v_cmd - kf[:, 0], label="v command - KF v")
    ax.axhline(0, color="k", lw=0.7)
    finish_axis(ax, "m/s", "Forward-speed error")

    ax = axes[1, 0]
    ax.plot(t, yaw_ref, "k--", lw=1.2, label="yaw reference")
    ax.plot(t, yaw_cmd, "k:", lw=1.2, label="yaw command")
    ax.plot(t, kf[:, 1], lw=1.2, label="KF omega")
    ax.plot(t, yaw_enc_rate, lw=0.8, alpha=0.5, label="encoder yaw rate")
    finish_axis(ax, "rad/s", "Yaw rate")

    ax = axes[1, 1]
    ax.plot(t, yaw_err, label="PID error (refLim - omega)")
    ax.plot(t, yaw_ref - kf[:, 1], label="yaw reference - KF omega")
    ax.axhline(0, color="k", lw=0.7)
    finish_axis(ax, "rad/s", "Yaw-rate error")

    ax = axes[2, 0]
    ax.plot(t, ref_heading_deg, "k--", lw=1.2, label="desired heading")
    ax.plot(t, heading_deg, lw=1.2, label="KF total angle")
    ax.plot(t, enc_heading_deg, lw=1.0, alpha=0.8, label="encoder yaw")
    if meta["turn_duration"]:
        ax.axhline(np.rad2deg(target_angle), color="k", ls=":",
                   lw=0.9, label="target angle")
    finish_axis(ax, "deg", "Heading / total angle")

    ax = axes[2, 1]
    ax.plot(t, dist, lw=1.2, label="KF total distance")
    ax.plot(t, ref_distance, "k--", lw=1.2, label="integrated v reference")
    if target_distance:
        ax.axhline(target_distance, color="k", ls=":", lw=0.9,
                   label="target distance")
    finish_axis(ax, "m", "Distance")

    for ax in axes[-1, :]:
        ax.set_xlabel("Time (s)")

    out = out_dir / f"{path.stem}_tracking.png"
    fig.savefig(out, dpi=160)
    plt.close(fig)
    return out


def plot_control(path, data, meta, out_dir):
    t = data[:, C_TIME]
    state = data[:, C_STATE].astype(int)
    motor_v = data[:, C_MOTOR_V:C_MOTOR_V + 2]
    control = data[:, C_CONTROL:C_CONTROL + 2]
    setpoint = data[:, C_SETPOINT:C_SETPOINT + 2]
    feedback = data[:, C_FEEDBACK:C_FEEDBACK + 2]
    pwm = data[:, C_PWM:C_PWM + 2]
    yaw_cmd = data[:, C_YAW_CMD]
    omega_ref = data[:, C_OMEGA_REF]
    yaw_err = data[:, C_YAW_ERR]
    head_err = data[:, C_HEAD_ERR]
    head_out = data[:, C_HEAD_OUT]
    if data.shape[1] > C_YAW_PID:
        yaw_pid = data[:, C_YAW_PID]
    else:
        yaw_pid = yaw_cmd - omega_ref

    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True,
                             constrained_layout=True)
    fig.suptitle(f"Control effort — {path.name}", fontsize=11)
    for ax in axes.flat:
        shade_phases(ax, t, state)

    ax = axes[0, 0]
    ax.plot(t, setpoint[:, 0], label="setpoint L")
    ax.plot(t, setpoint[:, 1], label="setpoint R")
    ax.plot(t, feedback[:, 0], "--", label="feedback L")
    ax.plot(t, feedback[:, 1], "--", label="feedback R")
    finish_axis(ax, "motor rad/s", "Wheel setpoints and feedback")

    ax = axes[0, 1]
    ax.plot(t, setpoint[:, 0] - feedback[:, 0], label="error L")
    ax.plot(t, setpoint[:, 1] - feedback[:, 1], label="error R")
    ax.axhline(0, color="k", lw=0.7)
    finish_axis(ax, "motor rad/s", "Wheel PI errors")

    ax = axes[1, 0]
    ax.plot(t, motor_v[:, 0], label="voltage L")
    ax.plot(t, motor_v[:, 1], label="voltage R")
    ax.plot(t, control[:, 0], "--", label="PI output L")
    ax.plot(t, control[:, 1], "--", label="PI output R")
    ax.axhline(0, color="k", lw=0.7)
    finish_axis(ax, "V", "Commanded motor voltage / PI output")

    ax = axes[1, 1]
    ax.plot(t, pwm[:, 0] / 4096.0 * 100.0, label="PWM L")
    ax.plot(t, pwm[:, 1] / 4096.0 * 100.0, label="PWM R")
    ax.axhline(0, color="k", lw=0.7)
    finish_axis(ax, "% of full scale", "Signed PWM request")

    ax = axes[2, 0]
    ax.plot(t, yaw_pid, label="yaw PID output")
    ax.plot(t, yaw_err, label="yaw-rate error")
    ax.axhline(0, color="k", lw=0.7)
    finish_axis(ax, "rad/s", "Yaw-rate PID output")

    ax = axes[2, 1]
    ax.plot(t, head_out, label="heading PID output")
    ax.axhline(0, color="k", lw=0.7)
    ax.set_ylabel("rad/s")
    ax.set_title("Heading PID output and error", loc="left")
    ax.grid(True, alpha=0.3)
    twin = ax.twinx()
    twin.plot(t, np.rad2deg(head_err), color="0.35", alpha=0.65,
              label="heading error (deg)")
    twin.set_ylabel("deg")
    lines, labels = ax.get_legend_handles_labels()
    lines2, labels2 = twin.get_legend_handles_labels()
    ax.legend(lines + lines2, labels + labels2, loc="best", fontsize=7)

    for ax in axes[-1, :]:
        ax.set_xlabel("Time (s)")

    out = out_dir / f"{path.stem}_control.png"
    fig.savefig(out, dpi=160)
    plt.close(fig)
    return out


def plot_estimator(path, data, meta, out_dir):
    t = data[:, C_TIME]
    state = data[:, C_STATE].astype(int)
    kf = data[:, C_KF:C_KF + 5]
    innov = data[:, C_INNOV:C_INNOV + 3]
    kf_mask = data[:, C_KF_MASK].astype(int)
    flags = data[:, C_FLAGS].astype(int)

    fig, axes = plt.subplots(2, 2, figsize=(14, 7), sharex=True,
                             constrained_layout=True)
    fig.suptitle(f"Estimator — {path.name}", fontsize=11)
    for ax in axes.flat:
        shade_phases(ax, t, state)

    ax = axes[0, 0]
    ax.plot(t, kf[:, 0], label="KF v")
    ax.set_ylabel("v (m/s)")
    ax.set_title("KF velocity and yaw rate", loc="left")
    ax.grid(True, alpha=0.3)
    twin = ax.twinx()
    twin.plot(t, kf[:, 1], color="C1", label="KF omega")
    twin.set_ylabel("omega (rad/s)")
    lines, labels = ax.get_legend_handles_labels()
    lines2, labels2 = twin.get_legend_handles_labels()
    ax.legend(lines + lines2, labels + labels2, loc="best", fontsize=7)

    ax = axes[0, 1]
    ax.plot(t, kf[:, 3], label="b_a")
    ax.axhline(0, color="k", lw=0.7)
    finish_axis(ax, "m/s^2", "Accelerometer bias")

    ax = axes[1, 0]
    ax.plot(t, innov[:, 0], label="gyro")
    ax.plot(t, innov[:, 1], label="wheel v")
    ax.plot(t, innov[:, 2], label="wheel omega")
    ax.axhline(0, color="k", lw=0.7)
    finish_axis(ax, "mixed units", "KF innovations")

    ax = axes[1, 1]
    ax.axis("off")
    duration = t[-1] - t[0]
    healthy = 100.0 * np.mean((flags & (1 << 1)) != 0)
    gyro_rows = 100.0 * np.mean((kf_mask & 1) != 0)
    as5_rows = 100.0 * np.mean((flags & (1 << 2)) != 0)
    left_sat = 100.0 * np.mean((flags & (1 << 5)) != 0)
    right_sat = 100.0 * np.mean((flags & (1 << 6)) != 0)
    head_sat = 100.0 * np.mean((flags & (1 << 7)) != 0)
    yaw_sat = 100.0 * np.mean((flags & (1 << 8)) != 0)
    gyro_sat = 100.0 * np.mean((flags & (1 << 13)) != 0)
    text = (
        f"samples        {len(t)}\n"
        f"duration       {duration:.2f} s\n"
        f"log rate       {len(t) / max(duration, 1e-9):.0f} Hz\n"
        f"KF healthy     {healthy:.1f}%\n"
        f"gyro update    {gyro_rows:.1f}% of rows\n"
        f"AS5 valid      {as5_rows:.1f}% of rows\n"
    )
    if data.shape[1] > C_TOTAL_ANGLE:
        text += (f"final distance {data[-1, C_DIST]:.2f} m\n"
                 f"final angle    {np.rad2deg(data[-1, C_TOTAL_ANGLE]):.1f} deg\n")
    text += (
        f"\nwheel L/R sat  {left_sat:.1f}% / {right_sat:.1f}%\n"
        f"heading sat    {head_sat:.1f}%\n"
        f"yaw-rate sat   {yaw_sat:.1f}%\n"
        f"gyro sat       {gyro_sat:.1f}%"
    )
    ax.text(0.02, 0.98, text, va="top", ha="left",
            family="monospace", fontsize=9)
    ax.set_title("Status summary", loc="left")

    for ax in axes[-1, :]:
        ax.set_xlabel("Time (s)")

    out = out_dir / f"{path.stem}_estimator.png"
    fig.savefig(out, dpi=160)
    plt.close(fig)
    return out


def report_wheel_magnetics(data, ppr):
    if data.shape[1] < C_DIAAGC + 2:
        return
    for k, side in enumerate(("left", "right")):
        mag = data[:, C_MAG + k]
        angle = data[:, C_ANGLECOM + k]
        diag = data[:, C_DIAAGC + k].astype(int)
        valid_mag = mag[mag >= 0]
        magnitude = (f"{valid_mag.min():.0f}..{valid_mag.max():.0f} counts"
                     if valid_mag.size else "unavailable")
        encoder_step = np.diff(data[:, C_ENCODER + k])
        moving = encoder_step != 0
        angle_pairs = moving & (angle[:-1] >= 0) & (angle[1:] >= 0)
        changes = np.count_nonzero(angle_pairs & (np.diff(angle) != 0))
        ready = (diag >= 0) & ((diag & 0xF00) == 0x100)  # LF set, COF/MAGH/MAGL clear
        print(f"AS5 {side}: MAG {magnitude} ({valid_mag.size}/{len(mag)} rows), "
              f"DIAAGC ready and field OK {ready.sum()}/{len(diag)} rows, "
              f"angle changed in {changes}/{moving.sum()} moving intervals "
              f"({angle_pairs.sum()} with two valid frames).")
        if ppr:
            # A sampled shaft angle wraps every revolution. Allow one encoder
            # count of quantization margin when choosing unambiguous intervals.
            usable = angle_pairs & (np.abs(encoder_step) + 1 < ppr / 2)
            if usable.any():
                angle_step = (np.diff(angle)[usable] + 8192) % 16384 - 8192
                counts = encoder_step[usable]
                shaft_revs = np.sum(angle_step * np.sign(counts)) / 16384
                encoder_revs = np.sum(np.abs(counts)) / ppr
                print(f"  Native shaft travel / forward-positive quadrature travel "
                      f"{shaft_revs / encoder_revs:+.3f} over {usable.sum()} intervals "
                      f"(expected {(-1, 1)[k]:+d} with the configured mounting).")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", nargs="?", type=Path,
                        help="format-4 log (default: newest in src/logs)")
    parser.add_argument("--out-dir", type=Path, default=None,
                        help="output directory (default: log directory)")
    args = parser.parse_args()

    log_dir = Path(__file__).resolve().parents[1] / "logs"
    if args.log is None:
        path, data, meta = latest_log(log_dir)
    else:
        path = args.log
        data, meta = load_log(path)
    out_dir = args.out_dir or path.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"Selected log: {path}")
    print(f"Rows: {len(data)}, duration {data[-1, C_TIME] - data[0, C_TIME]:.2f} s")
    turns = ", ".join(f"{np.rad2deg(a):+.1f}°" for a in meta["turn_angles"])
    print(f"Targets: {meta['target_distance']:.2f} m, turns {turns}")
    report_wheel_magnetics(data, meta["ppr"])
    if 14 in data[:, C_STATE] and data[-1, C_STATE] == 2:
        dx = data[-1, C_X] - data[0, C_X]
        dy = data[-1, C_Y] - data[0, C_Y]
        # XY uses the persistent odometry frame. Undo its starting heading
        # to express the offset along the marked outbound and left axes.
        start_heading = data[0, C_KF + 2] - data[0, C_REL_HEADING]
        forward = dx * np.cos(start_heading) + dy * np.sin(start_heading)
        left = -dx * np.sin(start_heading) + dy * np.cos(start_heading)
        print(f"Estimated endpoint: X {100 * forward:+.1f} cm (forward), "
              f"Y {100 * left:+.1f} cm (left), distance {100 * np.hypot(dx, dy):.1f} cm")
        final_turn = np.flatnonzero(data[:, C_STATE] == 17)
        if final_turn.size:
            shift = data[-1, C_X:C_Y + 1] - data[final_turn[0], C_X:C_Y + 1]
            print(f"Estimated displacement during final turn and hold: "
                  f"{100 * np.linalg.norm(shift):.1f} cm, final heading relative to start "
                  f"{np.rad2deg(data[-1, C_REL_HEADING]):+.2f}°")

    outputs = [
        plot_tracking(path, data, meta, out_dir),
        plot_control(path, data, meta, out_dir),
        plot_estimator(path, data, meta, out_dir),
    ]
    for output in outputs:
        print(f"Saved: {output}")


if __name__ == "__main__":
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        main()
