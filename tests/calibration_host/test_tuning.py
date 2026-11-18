"""Measured-angle, negative-step, fresh-sample and encoder-wrap regressions."""
import sys
import contextlib
import io
from pathlib import Path

sys.dont_write_bytecode = True
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src/scripts"))
from analyze_tuning import rotation, capture, encoder_position, measurement_time
from analyze_calibration import _step_metrics, analyze_noise
from analyze_stream import analyze_stream


def main():
    t = np.arange(1001) * 0.01
    stream = np.zeros((len(t), 21))
    stream[:, 0] = t
    stream[:, 19:21] = ((t + 1) * 1e6)[:, None]
    stream[:, 5:8] = [0.1, -0.2, 0.3]
    # Reported gyro axis Z carries 360/2.4 deg over six seconds.
    stream[(t >= 2) & (t < 8), 7] += 25
    r = rotation(stream, 360)
    assert r["axis"] == 2 and r["sign"] == 1
    assert abs(r["scale"] - 2.4) < 1e-5
    r = rotation(stream, -360)
    assert r["sign"] == -1 and abs(r["scale"] - 2.4) < 1e-5
    # A stale row must not contribute even if its value is corrupt.
    stream[350, 19] = stream[349, 19]
    stream[350, 7] = 1e5
    assert abs(rotation(stream, 360)["scale"] - 2.4) < 1e-5
    assert np.allclose(measurement_time(np.array([2**32-10, 0, 10])), [0, 1e-5, 2e-5])

    y = -10 * (1 - np.exp(-t / .08))
    metrics = _step_metrics(t, y, -10, 0, np.zeros(len(t)), 0, len(t))
    assert .15 < metrics["rise"] < .20  # 10-90% rise = ln(9)*tau

    config = {"radius": .0315, "gear": 9.6, "base": .14, "ppr": 48, "limit": 7}
    d = np.zeros((len(t), 55))
    d[:, 0] = t
    d[:, 40] = 10
    d[:, 43] = (32760 + np.arange(len(t)) + 32768) % 65536 - 32768
    d[:, 44] = (-32760 - np.arange(len(t)) + 32768) % 65536 - 32768
    pos = encoder_position(d, config)
    assert pos[-1, 0] > 0 and pos[-1, 1] < 0
    assert abs(pos[-1, 0] + pos[-1, 1]) < 1e-12

    rng = np.random.default_rng(3)
    d[:, 51:54] = ((t + 1)*1e6)[:, None]
    d[:, 48] = rng.normal(0, np.deg2rad(.17), len(t))
    d[:, 17] = rng.normal(0, .12/9.80665, len(t))
    d[:, 49] = rng.normal(0, .0145, len(t))
    d[:, 50] = d[:, 48] + .9 + rng.normal(0, .18, len(t))
    d[:, 54] = 16  # encoder source, despite no fresh AS5 flags
    noise = capture(d, "noise", config)
    assert abs(noise["stationary"]["sigma_gyro_dps"] - .17) < .01
    assert len(noise["noise"]) == 1
    assert noise["noise"][0]["source"] == "encoder"
    assert abs(noise["noise"][0]["yaw_bias"] - .9) < .02
    assert abs(noise["noise"][0]["yaw_residual_sigma"] - .18) < .02
    # Existing noise tools must also use the calibrated yaw mapping.
    with contextlib.redirect_stdout(io.StringIO()):
        mapped = analyze_noise(t, d)
        stationary_stream = np.zeros((len(t), 21))
        stationary_stream[:, 0] = t
        stationary_stream[:, 7] = rng.normal(0, .1, len(t))
        stationary_stream[:, 19:21] = ((t + 1)*1e6)[:, None]
        mapped_stream = analyze_stream(t, stationary_stream, yaw_axis=2, yaw_sign=1, yaw_scale=2)
    assert abs(mapped["checks"]["gyro yaw (deg/s)"]["measured"] - .17) < .01
    assert abs(mapped_stream["sigma"]["gyro_yaw_deg"] - .2) < .02

    # Positive and negative yaw steps include rate-limited references and
    # explicit correction saturation; wheel voltage headroom is separate.
    d[:, 40] = np.where(t < 1, 10, np.where(t < 5, 11, 12))
    ref = np.where(t < 1, 0, np.where(t < 5, .2, -.2))
    d[:, 45:48] = ref[:, None]
    d[:, 48] = ref
    d[:, 54] = np.where(t < 5, 16, 16 | 256)
    rows = capture(d, "yaw", config)["steps"]
    assert len(rows) == 2 and rows[-1]["target"] < 0
    assert rows[0]["saturation"] == 0 and rows[1]["saturation"] == 1
    assert rows[1]["wheel_saturation"] == 0
    print("PASS tuning analysis: measured gyro scale, fresh samples, signed steps, count wrap, noise sources")


if __name__ == "__main__":
    main()
