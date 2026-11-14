#!/usr/bin/env python3
"""Plot the stationary estimator, the outer-loop commands, and wheel setpoints.

The Basebot log header documents the format-4 columns:

    1      time (s)
    2      robot state
    3      forward velocity reference (m/s)
    30-31  wheel controller setpoint (left, right) (motor rad/s)
    32     yaw-rate reference (rad/s)
    33-37  estimator [v, omega, dead-reckoned theta, b_a, reserved]
    38     forward-velocity control error (vCmd - v) (m/s)
    39     yaw-rate PID error (omegaRefLim - omega) (rad/s)
    40     outer-loop forward-velocity command (m/s)
    41     outer-loop yaw-rate command (rad/s)

The static estimator does not propagate a covariance, so there is no
covariance plot. With no argument the newest log in ../logs is used.
"""

import argparse
import warnings
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

STATE_LABELS = [
    r"$v$ (m/s)",
    r"$\omega$ (rad/s)",
    r"$\theta$ dead-reckoned (rad)",
    r"$b_a$ (m/s$^2$)",
    r"reserved",
]
STATE_NAMES = ["v", "omega", "theta", "b_a", "reserved"]

COL_TIME = 0
COL_STATE = 1
COL_DESIRED_V = 2
COL_MOTOR_VEL_L = 7
COL_MOTOR_VEL_R = 8
COL_SETPOINT_L = 29
COL_SETPOINT_R = 30
COL_DESIRED_OMEGA = 31
COL_KF_STATE = 32
N_STATE = 5
COL_V_ERROR = 37
COL_OMEGA_ERROR = 38
COL_V_COMMAND = 39
COL_OMEGA_COMMAND = 40
N_COLUMNS = 41


def latest_log(directory):
    logs = sorted(directory.glob("*.log"), key=lambda p: p.stat().st_mtime)
    if not logs:
        raise SystemExit("No .log files found in %s" % directory)
    return logs[-1]


def load_log(path):
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        data = np.loadtxt(path, comments="%", ndmin=2)
    if data.size == 0:
        raise SystemExit("%s contains no data rows (only header/comments)." % path)
    if data.shape[1] < N_COLUMNS:
        raise SystemExit(
            "%s has %d columns, but the format-4 log needs %d (columns "
            "1-41).\nThis looks like a log from older firmware."
            % (path, data.shape[1], N_COLUMNS)
        )
    return data


def unpack(data):
    t = data[:, COL_TIME]
    state = data[:, COL_KF_STATE:COL_KF_STATE + N_STATE]
    return t, state


def active_spans(t, active):
    spans = []
    start = None
    for i, flag in enumerate(active):
        if flag and start is None:
            start = t[i]
        elif not flag and start is not None:
            spans.append((start, t[i]))
            start = None
    if start is not None:
        spans.append((start, t[-1]))
    return spans


def shade_run(ax, t, data):
    active = data[:, COL_STATE] > 0
    for i, (start, stop) in enumerate(active_spans(t, active)):
        ax.axvspan(
            start, stop, color="tab:green", alpha=0.07, lw=0,
            label="run active" if i == 0 else None,
        )


def plot_state(t, state, data):
    fig, axes = plt.subplots(N_STATE, 1, figsize=(12, 10), sharex=True)
    for k, ax in enumerate(axes):
        color = "C%d" % k
        ax.plot(t, state[:, k], color=color, lw=1.0, label="estimate")
        ax.set_ylabel(STATE_LABELS[k])
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper left", fontsize=7, frameon=False)

    axes[0].plot(t, data[:, COL_DESIRED_V], "k--", lw=1.0, label="reference")
    axes[0].legend(loc="upper left", fontsize=7, frameon=False)
    axes[1].plot(t, data[:, COL_DESIRED_OMEGA], "k--", lw=1.0,
                 label="reference")
    axes[1].legend(loc="upper left", fontsize=7, frameon=False)
    axes[0].set_title("Stationary estimator state")
    axes[-1].set_xlabel("time (s)")
    fig.tight_layout()
    return fig


def plot_control(t, data):
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)

    axes[0].plot(t, data[:, COL_DESIRED_V], "k--", label="reference")
    axes[0].plot(t, data[:, COL_KF_STATE], color="C0", label="estimate")
    axes[0].plot(t, data[:, COL_V_COMMAND], color="C2", label="command")
    axes[0].plot(t, data[:, COL_V_ERROR], color="C3", lw=0.8, label="error")
    axes[0].set_ylabel("forward speed (m/s)")
    axes[0].set_title("Outer-loop velocity and yaw-rate control")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best", fontsize=8)

    axes[1].plot(t, data[:, COL_DESIRED_OMEGA], "k--", label="reference")
    axes[1].plot(t, data[:, COL_KF_STATE + 1], color="C0", label="estimate")
    axes[1].plot(t, data[:, COL_OMEGA_COMMAND], color="C2", label="command")
    axes[1].plot(t, data[:, COL_OMEGA_ERROR], color="C3", lw=0.8,
                 label="error")
    axes[1].set_ylabel("yaw rate (rad/s)")
    axes[1].set_xlabel("time (s)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="best", fontsize=8)

    fig.tight_layout()
    return fig


def plot_setpoint(t, data):
    fig, ax = plt.subplots(figsize=(11, 5))
    setpoint_l = data[:, COL_SETPOINT_L]
    setpoint_r = data[:, COL_SETPOINT_R]
    vel_l = data[:, COL_MOTOR_VEL_L]
    vel_r = data[:, COL_MOTOR_VEL_R]

    shade_run(ax, t, data)
    ax.plot(t, setpoint_l, color="C0", lw=1.6, label="setpoint left")
    ax.plot(t, setpoint_r, color="C1", lw=1.6, label="setpoint right")
    ax.plot(t, vel_l, color="C0", lw=0.7, alpha=0.6, ls="--",
            label="encoder velocity left")
    ax.plot(t, vel_r, color="C1", lw=0.7, alpha=0.6, ls="--",
            label="encoder velocity right")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("motor speed (rad/s)")
    ax.set_title("Wheel-speed controller setpoints")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    return fig


def print_summary(path, t, state, data):
    active = data[:, COL_STATE] > 0
    spans = active_spans(t, active)

    print("Log:      %s" % path)
    print("Samples:  %d over %.4f s" % (len(t), t[-1] - t[0]))
    if spans:
        print("Run active: " + ", ".join("%.3f-%.3f s" % s for s in spans))
    print("Final state estimate:")
    for k in range(N_STATE):
        print("  %-6s = %12.6g" % (STATE_NAMES[k], state[-1, k]))
    for name, col in (("left", COL_SETPOINT_L), ("right", COL_SETPOINT_R)):
        values = data[:, col]
        print("Setpoint %-5s range: %.1f .. %.1f motor rad/s" % (
            name, values.min(), values.max()))
    for name, col in (("v", COL_V_COMMAND), ("omega", COL_OMEGA_COMMAND)):
        values = data[:, col]
        print("Command %-6s range: %.4f .. %.4f" % (
            name, values.min(), values.max()))


def main():
    default_logs = Path(__file__).resolve().parents[1] / "logs"
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "log", nargs="?", type=Path,
        help="log file to analyze (default: newest in %s)" % default_logs,
    )
    parser.add_argument(
        "--outdir", type=Path,
        help="save figures as PNG in this directory instead of showing them",
    )
    args = parser.parse_args()

    path = args.log or latest_log(default_logs)
    data = load_log(path)
    t, state = unpack(data)

    print_summary(path, t, state, data)

    fig_state = plot_state(t, state, data)
    fig_control = plot_control(t, data)
    fig_setpoint = plot_setpoint(t, data)

    if args.outdir:
        args.outdir.mkdir(parents=True, exist_ok=True)
        stem = path.stem
        outputs = [
            (fig_state, "state"),
            (fig_control, "control"),
            (fig_setpoint, "setpoint"),
        ]
        for fig, suffix in outputs:
            out = args.outdir / (stem + "_" + suffix + ".png")
            fig.savefig(out, dpi=150)
            print("Saved %s" % out)
    else:
        plt.show()


if __name__ == "__main__":
    main()
