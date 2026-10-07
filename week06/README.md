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

## Composite robot: velocity and turn-rate commands

`week06_robot_model.slx` exposes two scalar inputs: commanded forward velocity
(m/s) and commanded turn rate (rad/s). Its reusable masked **Robot** subsystem
outputs four scalar signals, ordered as signed distance (m), actual velocity
(m/s), unwrapped turn angle (rad), and actual turn rate (rad/s). The scope and
`out.robotOutputs` timeseries use the same output order.

All physical, controller and initial-state defaults are defined in MATLAB:

```matlab
addpath('week06')
[wheels, robot] = robot_model_defaults();
robot.trackWidth = 0.14;       % wheel spacing, m
robot.initialDistance = 0;    % signed distance, m
robot.initialAngle = 0;       % heading, rad
wheels(1).Kp = 2.03180408641676;
wheels(2).Kp = 0.766619928019031;
model = configure_robot_model(wheels, robot);
```

Alternatively run `week06/init_robot_model.m` for the saved week06 defaults.
`configure_robot_model` installs parameters in the model workspace and applies
MATLAB-owned solver settings. Changes are in memory; use `save_system(model)`
to persist them. Reducing a wheel's Ts also requires reducing `robot.maxStep`
to at most `min([wheels.Ts])/10`. Saved defaults permit opening the model in a
clean session without rerunning identification or tuning.

Example external commands, when simulation is desired:

```matlab
t = [0; 0.2; 0.201; robot.stopTime];
commands = [t, [0; 0; 0.1; 0.1], [0; 0; 0.5; 0.5]];
input = Simulink.SimulationInput(model);
input = input.setExternalInput(commands);
out = sim(input);
```

Steering uses `leftRef = velocityRef - trackWidth*turnRateRef/2` and
`rightRef = velocityRef + trackWidth*turnRateRef/2`. Robot motion comes from
actual wheel speeds: velocity is their average and turn rate is right minus
left divided by track width. Positive turns are counterclockwise. Reversing
decreases signed distance. The unequal tuned P loops can produce residual
heading drift even with a zero turn command. No chassis or slip dynamics are
added. The wheel radius and gearing retain the values used for tuning.

Each wheel also exposes MATLAB parameters `initialMotorSpeed` (motor rad/s),
`initialCurrent` (A), `initialSensorSpeed` (delayed motor rad/s), and
`diagnosticVoltageScale`. The Robot mask accepts `wheelParameters` and
`robotParameters` structures: when copying Robot, define those structures in
the destination workspace and set its two mask fields to their names. Configure
the destination solver suitably for the wheel sample times.

Run `build_robot_model` to regenerate the model from the saved week06 wheel
blocks and MATLAB defaults. Close the robot model before rebuilding; rebuilding
overwrites the generated file. Fixed factors such as averaging by `1/2` are
mathematical constants, while adjustable values use workspace expressions.

### Command response and nominal stability experiment

Run `run('week06/simulate_robot_response.m')` from the repository root. Edit
`commandSchedule` in the script to change the held [time, velocity, turn rate]
commands, and override `wheels` or `robot` before configuration to investigate
parameter changes. Input interpolation is disabled in memory for this run.
The script logs both wheel voltages, plots all four robot outputs and the
wheel responses, and saves `response.png`, `response.mat`, `segments.csv` and
`stability.csv` in `week06/robot_results`. It requires Simulink and Control
System Toolbox. It leaves the model open with experiment settings in memory;
close without saving to restore its saved configuration.

The default 20 s experiment covers rest, straight driving, turning in place,
combined commands in both turn directions, reverse driving, and stopping.
With the saved parameters and Ts=1 ms, all held segments settled. The nominal
ZOH + one-sample latency linear model gives maximum pole magnitudes 0.94242
(left) and 0.86668 (right), both below one. Phase margins are 15.515 degrees
and 36.962 degrees; gain margins are 2.4083 dB and 6.1229 dB. The left loop
therefore has the smaller robustness margin. These are actual achieved
margins, not the tuner's requested 60-degree target.

At a straight 0.1 m/s command, predicted/simulated steady robot speed is
0.099145 m/s and residual turn rate is -0.0047246 rad/s (about -0.271 deg/s).
Unequal P-controller steady errors cause this drift. Steering does not add
feedback between the independent wheel loops, so nominal wheel-loop stability
is preserved in the composite speed response. Distance and heading integrate
the resulting motion and can grow under constant commands or accumulated
tracking error; there is no outer position/heading regulator. The linear
margins use the tuning approximation; the actual sampled, saturated model is
also simulated. Neither this experiment nor nominal margins guarantee
stability for different sample times, gains, loading, or real traction.
