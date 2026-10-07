# P-controlled wheel

## Basebot with measured left/right motors

### Tune and save P gains

```matlab
addpath('week06')
selected = tune_basebot_p;
```

The tuner uses Control System Toolbox `pidtune(...,'P',...)` on each
motor's voltage-to-motor-angular-speed transfer function, discretized with
the DA zero-order hold and one sample of sensor latency. It sweeps 24
requested crossover frequencies from 20 to 1000 rad/s with a requested
60-degree phase margin. Each stable candidate is checked in the actual
Simulink model, including saturation and its continuous transport delay.

The chosen gain minimizes integral absolute speed error over the 0.1 m/s
step experiment among tested candidates with steady error below 2%,
reference overshoot below 15%, and final-window variation below 1e-5 m/s.
This is a constrained candidate search, not a proof of a global optimum.
Adjust these limits in `tune_basebot_p.m` to change the tradeoff.

Gains are saved in `basebot_p_gains.mat`, the model workspace and
`basebot_motor_parameters.csv`. `basebot_p_selected.csv` records their
performance; `basebot_p_tuning_sweep.csv` contains all simulated candidates.
`run_basebot_test` automatically loads saved gains and rejects them if motor
parameters or Ts have changed. Set `useTunedGains=false` for manual gains.

Verified selected gains at Ts=1 ms:

| Motor | Kp (V/(rad/s)) | Error (%) | Reference overshoot (%) |
|---|---:|---:|---:|
| Left | 2.0318041 | 0.52386 | 11.457 |
| Right | 0.76661993 | 1.1853 | 14.811 |

Both settle; startup voltage saturation is permitted by this objective.
These gains are tuned for this model and reference. Validate on hardware
before applying them broadly, especially if sample time or loading changes.

```matlab
addpath('week06')
summary = run_basebot_test;
open_system('week06_basebot_test')
```

`run_basebot_test.m` runs the root `analysis.m` on the 6 V step from
`src/logs/device-monitor-260909-113357.log`, loads each motor's R, L, Kb,
D and reflected J, and assumes Kt=Kb in SI units. The inertia includes
half of the 0.927 kg robot mass for each wheel. This uses the root analysis;
the separate notebook analysis currently specifies 927 kg.

Edit Kp, Ts, referenceSpeed and the simulation duration at the top of the
test script. The generated `week06_basebot_test.slx` has two masked Wheel
blocks referring to `wheels(1)` and `wheels(2)` in its model workspace.
Saved parameters allow it to run directly after reopening. The two speed
loops are independent; chassis and traction dynamics are not included.

Results in this folder include `basebot_motor_parameters.csv`,
`basebot_steady_state.csv`, `basebot_test_results.mat` and
`basebot_steady_state.png`. Speed and error are averaged over uniformly
sampled points in the last 0.5 s of a 3 s simulation. The report also gives
final speed variation, saturation fraction and the analytic unsaturated
prediction: `v = reference*Kp/(Kp + Kb + R*D/Kt)`.
The script checks settling, voltage limits and agreement with this prediction.

Original baseline in R2026a with Kp=0.05 and Ts=1 ms for both motors
(saved results now use tuned gains):

| Wheel | Reference (m/s) | Steady speed (m/s) | Error (m/s) | Error (%) |
|---|---:|---:|---:|---:|
| Left | 0.1 | 0.082372 | 0.017628 | 17.628 |
| Right | 0.1 | 0.084465 | 0.015535 | 15.535 |

Both responses settled without voltage saturation. These are predictions
from the identified model. The measured left steady current is -0.0385 A;
the analysis takes absolute values when estimating damping. Review current
offset/sign calibration before treating that estimate as a physical constant.

From the repository root in MATLAB:

```matlab
run('week06/init_week06.m')
out = sim(model);
plot(out.velP.Time, out.velP.Data(:,[1 4]))
```

Edit `wheel`, `referenceSpeed` and `stepTime` in `init_week06.m` and rerun
the script. It assigns parameters to the model workspace, so it works even
when called from a function. The saved model also contains defaults and can
run immediately in a clean session. Script changes are in memory; rerun the
script after reopening the model, or save the model to persist them.

The masked **Wheel** subsystem contains the P controller, ±9 V saturation,
DA/AD holds, sensor delay and continuous DC motor. Its input is reference
linear speed (m/s); output 1 is actual linear speed (m/s). Output 2 and
`out.velP` contain `[reference_mps, voltage_V/10, motor_radps,
velocity_mps, current_A]`. Double-click Wheel to edit its parameters;
use Look Under Mask to inspect its feedback loop.

Copy Wheel into a robot model and connect each reference to your steering
logic. Each copy has its own integrator states and mask parameters. Define
an array of parameter structs in the parent model workspace (for example,
`wheels = repmat(wheel,1,4)`) and set each mask field to the corresponding
expression, such as `wheels(2).R`, `wheels(2).Kp`, and `wheels(2).Ts`.
The parent model must define those structs; copied blocks initially refer
to `wheel`. Use separate scalar blocks for wheels with different parameters.
Choose the parent solver maximum step small enough for the smallest Ts
(the supplied model uses Ts/10).

The defaults reproduce week05: a 0.1 m/s reference settles at 0.08196721 m/s.
P control leaves 0.01803279 m/s steady-state error. Kp uses motor angular
speed error, in V/(rad/s). Ts controls sampling and sensor latency; increasing
it can destabilize the loop. This is a speed model, without steering-angle,
load-torque, traction or robot-body dynamics.

Run `addpath('week06'); verify_week06` to verify defaults, script overrides,
reverse motion, agreement with week05 and two independently parameterized
wheel copies. Verification requires MATLAB and Simulink.
