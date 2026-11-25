"""Fit the entry pivot and validate the exit pivot in a full-route format-4 log.

Example: python src/scripts/fit_lecture10_pivots.py path/to/log --candidate 0.85 0.65 1.40
"""
import argparse
from pathlib import Path

import numpy as np
from plot_latest_performance import C_HEAD_ERR, C_STATE, C_YAW_REF, load_log, plt


def pivot_samples(data, phase):
    t = data[:, 16] * 1e-6  # wall clock from capture timestamp (column 17)
    rows = np.flatnonzero(data[:, C_STATE] == phase)
    if len(rows) < 3 or not np.all(np.diff(rows) == 1):
        raise ValueError(f"Need a contiguous pivot in phase {phase}")
    # Include the preceding arc deceleration when predicting the exit pivot.
    window = slice(np.searchsorted(t, t[rows[0]] - 2.5), rows[-1] + 1)
    return t[window] - t[rows[0]], data[window, C_YAW_REF], data[window, C_HEAD_ERR]


def error_response(t, rate, tau):
    # Response of heading error to a unit step in requested rate: A*t*exp(-t/tau).
    # Sum the response to each recorded rate increment using its capture time.
    jumps = np.flatnonzero(np.diff(rate)) + 1
    age = np.maximum(t[:, None] - t[jumps], 0.0)
    steps = rate[jumps] - rate[jumps - 1]
    return np.sum(age * np.exp(-age / tau) * steps, axis=1)


def fit_entry(t, rate, error):
    pivot = t >= 0
    best_mse = np.inf
    best = None
    for tau in np.arange(0.05, 0.805, 0.005):
        basis = error_response(t, rate, tau)[pivot]
        power = basis @ basis
        if power == 0:
            raise ValueError("Entry pivot has no rate excitation")
        gain = (basis @ error[pivot]) / power
        mse = np.mean((gain * basis - error[pivot]) ** 2)
        if mse < best_mse:
            best_mse, best = mse, (tau, gain)
    return best


def simulate_pivot(angle, peak, rise, fall, tau, gain):
    # Exact area of a clockwise trapezoid. The measured gate is 1 deg and 0.08 rad/s.
    if not np.all(np.isfinite([peak, rise, fall])) or peak <= 0 or min(rise, fall) < 0:
        raise ValueError("Use a positive finite peak and nonnegative finite ramp times")
    end = angle / peak + (rise + fall) / 2
    if end < rise + fall:
        raise ValueError("Pivot needs a positive peak and ramps that fit the sweep")
    dt = 0.002
    t = np.arange(0, end + 3, dt)
    rate = np.where(t < end, -peak, 0.0)
    if rise > 0:
        rate *= np.clip(t / rise, 0, 1)
    if fall > 0:
        rate *= np.clip((end - t) / fall, 0, 1)
    error = gain * np.convolve(np.diff(rate, prepend=0), t * np.exp(-t / tau))[:len(t)]
    heading = np.r_[0, np.cumsum(rate[:-1]) * dt] - error
    yaw = rate - np.gradient(error, dt)
    ready = (t >= end) & (np.abs(heading + angle) <= np.deg2rad(1)) & (np.abs(yaw) <= 0.08)
    return t, np.rad2deg(heading + angle), t[ready][0] if ready.any() else np.nan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--candidate", type=float, nargs=3, metavar=("PEAK", "RISE", "FALL"),
                        help="simulate a clockwise pivot from rest (rad/s, s, s)")
    parser.add_argument("--plot", type=Path, help="save measured and predicted heading errors")
    args = parser.parse_args()
    data, _ = load_log(args.log)
    pivots = [pivot_samples(data, phase) for phase in (21, 23)]
    tau, gain = fit_entry(*pivots[0])
    print(f"Heading error per unit rate step: {gain:.3f} * t * exp(-t/{tau:.3f})")
    if args.plot:
        fig, axes = plt.subplots(1, 2, sharey=True, figsize=(9, 3.5), layout="constrained")
    for index, (t, rate, error) in enumerate(pivots):
        pivot = t >= 0
        prediction = np.rad2deg(gain * error_response(t, rate, tau)[pivot])
        measured = np.rad2deg(error[pivot])
        name = ("Entry pivot (fit)", "Exit pivot (held out)")[index]
        print(f"{name}: RMS residual {np.mean((prediction - measured)**2)**0.5:.2f} deg; "
              f"zero-error prediction {np.mean(measured**2)**0.5:.2f} deg")
        if args.plot:
            ax = axes[index]
            ax.plot(t[pivot], measured, color="#24537a", label="Logged heading error")
            ax.plot(t[pivot], prediction, color="#a34020", ls="--", label="Model")
            ax.axhline(0, color="0.6", lw=0.7)
            ax.set(title=name, xlabel="Time since pivot entry (s)")
    if args.plot:
        axes[0].set_ylabel("Reference minus estimated heading (deg)")
        axes[1].legend(frameon=False)
        fig.savefig(args.plot, dpi=170)
        plt.close(fig)
    if args.candidate:
        # Lecture 10 fixed course, 0.30 m gap in a 1.00 m diameter circle.
        angle = np.arccos(0.30)
        for name, shape in [("Cycle 2", (0.5236, 0, 0)), ("Candidate", args.candidate)]:
            _, to_target, ready = simulate_pivot(angle, *shape, tau, gain)
            print(f"{name} prediction from rest: handoff {ready:.2f} s, "
                  f"peak target overshoot {max(0, -min(to_target)):.2f} deg")


if __name__ == "__main__":
    main()
