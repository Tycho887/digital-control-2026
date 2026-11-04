#!/usr/bin/env python3
"""Plot the Kalman estimate, its covariance, and the control setpoints.

The Basebot log header documents the columns:

    1      time (s)
    2      robot state
    30-31  controller setpoint (left, right) (motor rad/s)
    32-38  Kalman state [s, theta, v, omega, b_a, b_g, b_m]
    39-66  covariance lower triangle, row-major P00; P10 P11; ...; P66

With no argument the most recently modified log in ../logs is used.
"""

import argparse
import warnings
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

STATE_LABELS = [
    r"$s$ (m)",
    r"$\theta$ (rad)",
    r"$v$ (m/s)",
    r"$\omega$ (rad/s)",
    r"$b_a$ (m/s$^2$)",
    r"$b_g$ (rad/s)",
    r"$b_m$ (rad)",
]
STATE_NAMES = ["s", "theta", "v", "omega", "b_a", "b_g", "b_m"]

COL_TIME = 0
COL_STATE = 1
COL_SETPOINT_L = 29
COL_SETPOINT_R = 30
COL_MOTOR_VEL_L = 7
COL_MOTOR_VEL_R = 8
COL_KF_STATE = 31
COL_KF_COV = 38
N_STATE = 7
N_COV = N_STATE * (N_STATE + 1) // 2  # 28
N_COLUMNS = COL_KF_COV + N_COV        # 66

RUN_ACTIVE_STATES = (11, 12)


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
            "%s has %d columns, but state and covariance need %d (columns "
            "32-66 of the current firmware log).\nThis looks like a log from "
            "older firmware." % (path, data.shape[1], N_COLUMNS)
        )
    return data


def unpack(data):
    t = data[:, COL_TIME]
    state = data[:, COL_KF_STATE:COL_KF_STATE + N_STATE]

    cov = data[:, COL_KF_COV:COL_KF_COV + N_COV]
    P = np.zeros((data.shape[0], N_STATE, N_STATE))
    k = 0
    for i in range(N_STATE):
        for j in range(i + 1):
            P[:, i, j] = cov[:, k]
            P[:, j, i] = cov[:, k]
            k += 1

    return t, state, P


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
    active = np.isin(data[:, COL_STATE], RUN_ACTIVE_STATES)
    for i, (start, stop) in enumerate(active_spans(t, active)):
        ax.axvspan(
            start, stop, color="tab:green", alpha=0.07, lw=0,
            label="run active" if i == 0 else None,
        )


def plot_state_and_covariance(t, state, P):
    sigma = np.sqrt(np.clip(np.diagonal(P, axis1=1, axis2=2), 0.0, None))
    fig, axes = plt.subplots(
        N_STATE, 2, figsize=(12, 12), sharex=True,
        gridspec_kw={"width_ratios": [2.2, 1]},
    )

    for k, (ax_mean, ax_var) in enumerate(axes):
        color = "C%d" % k
        mean = state[:, k]
        band = 2.0 * sigma[:, k]
        ax_mean.plot(t, mean, color=color, lw=1.0, label="estimate")
        ax_mean.fill_between(
            t, mean - band, mean + band, color=color, alpha=0.25,
            lw=0, label=r"$\pm 2\sigma$",
        )
        ax_mean.set_ylabel(STATE_LABELS[k])
        ax_mean.grid(True, alpha=0.3)
        ax_mean.legend(loc="upper left", fontsize=7, frameon=False)

        var = np.diagonal(P, axis1=1, axis2=2)[:, k]
        positive = var > 0
        ax_var.semilogy(t[positive], var[positive], color=color, lw=1.0)
        ax_var.set_ylabel(r"$\mathrm{var}(%s)$" % STATE_NAMES[k], fontsize=8)
        ax_var.grid(True, alpha=0.3, which="both")
        ax_var.tick_params(labelsize=7)

    axes[0, 0].set_title("Kalman state estimate with 2-sigma band")
    axes[0, 1].set_title("Covariance diagonal $P_{ii}$")
    for ax in axes[-1]:
        ax.set_xlabel("time (s)")
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


def print_summary(path, t, state, P, data):
    sigma = np.sqrt(np.clip(np.diagonal(P, axis1=1, axis2=2), 0.0, None))
    active = np.isin(data[:, COL_STATE], RUN_ACTIVE_STATES)
    spans = active_spans(t, active)

    print("Log:      %s" % path)
    print("Samples:  %d over %.4f s" % (len(t), t[-1] - t[0]))
    if spans:
        print("Run active: " + ", ".join("%.3f-%.3f s" % s for s in spans))
    print("Final state estimate:")
    for k in range(N_STATE):
        print("  %-6s = %12.6g +/- %.3g" % (
            STATE_NAMES[k], state[-1, k], sigma[-1, k]))
    for name, col in (("left", COL_SETPOINT_L), ("right", COL_SETPOINT_R)):
        values = data[:, col]
        print("Setpoint %-5s range: %.1f .. %.1f motor rad/s" % (
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
    t, state, P = unpack(data)

    print_summary(path, t, state, P, data)

    fig_state = plot_state_and_covariance(t, state, P)
    fig_setpoint = plot_setpoint(t, data)

    if args.outdir:
        args.outdir.mkdir(parents=True, exist_ok=True)
        stem = path.stem
        state_png = args.outdir / (stem + "_state.png")
        setpoint_png = args.outdir / (stem + "_setpoint.png")
        fig_state.savefig(state_png, dpi=150)
        fig_setpoint.savefig(setpoint_png, dpi=150)
        print("Saved %s" % state_png)
        print("Saved %s" % setpoint_png)
    else:
        plt.show()


if __name__ == "__main__":
    main()
