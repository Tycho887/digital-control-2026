# Programmed robot trajectory and results

The simulation uses `week06_robot_model.slx` with the existing tuned wheel
controllers. Starting at rest at `(x, y) = (0, 0)` with zero yaw, the robot is
commanded to drive forward, turn left in place, and drive forward again.

## Planned sequence

| Time (s, rounded) | Command |
|---|---|
| 0–4.00 | Remain stationary |
| 4.00–7.33 | Drive 0.5 m forward at 0.15 m/s |
| 7.33–7.83 | Pause |
| 7.83–10.97 | Turn 90° counterclockwise at 0.5 rad/s |
| 10.97–11.47 | Pause |
| 11.47–14.81 | Drive another 0.5 m forward at 0.15 m/s |
| 14.81–15.81 | Stop and allow the wheel response to settle |

Segment durations are calculated from commanded distance/speed and angle/yaw
rate. With perfect tracking, the path forms an L and ends at `(0.5, 0.5)` m
with 1 m accumulated distance and 90° yaw.

## Interpreting the four plots

The [result figure](robot_results/programmed_trajectory.png) shows the simulated
wheel-model response as solid curves and perfect command tracking as dashed curves.
All four horizontal axes show time in seconds.

- **x (m):** position along the initial forward direction. It should increase
  during the first straight and remain approximately constant during the second.
- **y (m):** position along the initial left direction. It should remain near
  zero until the second straight, then increase.
- **Signed accumulated distance (m):** the integral of actual forward velocity.
  It increases on both forward segments and stays approximately flat during the
  turn and pauses. Reverse motion would subtract from it.
- **Unwrapped yaw (rad):** accumulated heading, positive counterclockwise.
  The turn should raise it from approximately zero to π/2 rad (90°).
  It is not reset when the turn finishes or wrapped at a full revolution.

The latest default simulation ended with:

| Quantity | Ideal | Simulated | Simulated − ideal |
|---|---:|---:|---:|
| x | 0.500 m | 0.518 m | +0.018 m |
| y | 0.500 m | 0.491 m | −0.009 m |
| Signed distance | 1.000 m | 0.991 m | −0.009 m |
| Yaw | 90.00° | 86.54° | −3.46° |

Small departures from the dashed curves reflect wheel transients and unequal
steady tracking errors in the two P controllers. Yaw drift during straight
driving comes from unequal actual wheel speeds. The final heading error also
causes x to increase slightly during the second straight. The sequence uses
timed commands without an outer position or heading feedback loop, so these
errors are not corrected. Results are model predictions of wheel odometry;
the model does not include wheel slip or external localization.

## Run or change the experiment

From the repository root in MATLAB:

```matlab
run('week06/simulate_programmed_trajectory.m')
```

Edit the `trajectory` structure near the top of
[the script](simulate_programmed_trajectory.m) to change the sequence settings.
The script saves the figure and [simulation data](robot_results/programmed_trajectory.mat).
In the MAT file, `time` is in seconds and columns of `actual` and `ideal` are
`[distance_m, yaw_rad, x_m, y_m]`. MATLAB and Simulink are required.
