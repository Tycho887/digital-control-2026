# Digital P-controller experiments

Open MATLAB with this folder as the Current Folder, then run:

```matlab
open_system('basebot_model_sampling.slx')
run_sampling_experiments
```

The saved model includes its parameters in the model workspace and can also
run directly using Simulink's Run button. Double-click **Motor** to inspect
the electrical and mechanical blocks copied from `motor_model_23b.slx`.

## Change an experiment

Edit the settings at the top of `run_sampling_experiments.m`: `Ts`, `Kp`,
reference speed, step time, simulation durations and `sampleTimes`.
Set `runSweep=false` for a single run. Script overrides apply only to that
simulation; saved model defaults stay available for runs from the editor.
To change parameters for the Run button, use Model Explorer →
basebot_model_sampling → Model Workspace.

The motor parameters are L=0.001 H, R=1 ohm, Kt=Kb=0.01,
J=1e-5 kg m² and D=1e-5 N m s/rad. Gear ratio is 9.68 and wheel radius
is 0.03 m. The controller operates on motor angular velocity error, so Kp
has units V/(rad/s). Both Zero-Order Holds use `Ts`, and `detect_delay`
is a continuous Transport Delay of `Ts` seconds, initially outputting zero.
The plant stays continuous. The voltage limit is ±9 V.

## Results

The baseline simulation output is `aaa`. `aaa.velP` is a timeseries with
columns `[reference_mps, applied_voltage_V/10, velocity_mps]`.
Unscaled timeseries are `aaa.appliedVoltage`, `aaa.omega`,
`aaa.motorCurrent`, and `aaa.sampledFeedback` (rad/s).
The script saves plots and MAT files in `sampling_results`, plus
`sampling_summary.csv` with measurements for the sweep.

Summary statistics use uniform resampling. Steady-state error is the reference
minus mean velocity in the final 0.2 seconds; final-window variation is its
maximum minus minimum. Overshoot is the maximum excess above the reference
after the step, in m/s. Saturation fraction is the fraction of time after the
step with applied voltage at either limit. For oscillating responses, the
reported steady-state error is just a final-window mean, not a settled value.

Verified in MATLAB R2026a: the baseline Ts=1 ms, Kp=0.05 response settles at
0.08196721 m/s for a 0.1 m/s reference. The P-controller leaves a steady-state
error of 0.01803279 m/s. The copied motor matched the original simulated
angular velocity and current at all comparison points.
In the 2-second sweep, 20 ms produces decaying oscillations; 30–50 ms produces
large sustained oscillations and voltage saturation. This brackets the
observed transition between 20 and 30 ms for these settings; it does not
establish an exact stability boundary or an acceptable performance threshold.
Refine `sampleTimes` within that interval and adjust the duration as needed.

## Rebuild and validate

Close the new model before rebuilding. Rebuilding replaces its saved file:

```matlab
build_sampling_model
verify_sampling_model
```

The builder preserves the supplied model. Validation checks sample times,
delay, baseline output, voltage limits, and plant equivalence against the
original model using the original voltage-step sequence. It creates a
temporary in-memory test model and closes it afterward.
