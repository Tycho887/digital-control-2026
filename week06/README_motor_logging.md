# Checking the motor identification logs

The updated `src/src/basebot_6.ino` keeps columns 1–16 compatible with
`analysis.m` and appends columns 17–24. Current is now printed with six
decimal places, and each calibrated current is derived from the same raw
ADC reading that is logged.

| Columns | Signal |
|---|---|
| 17 | Actual capture timestamp, microseconds since start |
| 18–19 | Raw 12-bit current ADC counts, left A1 / right A0 |
| 20–21 | Signed PWM requests, forward-positive; full scale 4096 |
| 22 | Driver bits: enable left/right in bits 0/1; sleeping left/right in bits 2/3 |
| 23–24 | Encoder velocity estimate delay, seconds |

The header records current scale/offset, PWM frequency, encoder pulses per
revolution, gear ratio and wheel radius. Unused private log fields were
removed so the fixed 400 kB buffer still holds the full three-pulse capture.
No current calibration or motor-control behavior was changed.

After uploading the firmware, capture the existing sequence and save the
serial output as a new log. Check the raw ADC baseline while the motors
are stationary before each pulse. Use its mean as a candidate zero-current
reference: `Ileft = -(ADCleft-zeroLeft)*0.0045`,
`Iright = (ADCright-zeroRight)*0.0045`. Compare the zero readings before and
after the pulses to assess offset drift. The legacy offsets were explicitly
documented as belonging to Tania; they may not fit this robot. Check the
0.0045 A/count scale against an independent current measurement too.

Use column 17 differences to check missed samples and actual current rise
timing. ADC reads occur sequentially after motor.tick; they are not
synchronized to PWM, so switching ripple may influence peak current.
At 1 ms sampling, the estimated electrical time constants of roughly
1.2 ms are poorly resolved. A peak-current estimate of R can therefore be
biased by sampling and motor acceleration, even with these added signals.

Columns 4–5 are requested voltages, not terminal measurements. The driver
adds +/-0.4 V compensation, divides by battery voltage minus an assumed
1 V loss, and uses 11.1 V when battery voltage is below 5.5 V. PWM request
and battery readings expose these assumptions but cannot determine actual
terminal voltage. PWM requests are logged before any hardware clipping;
values beyond +/-4096 indicate a request outside the nominal range.
An independent terminal-voltage measurement is needed to validate R/Kb.

Compare motor speed with changes in encoder counts (columns 6–7) divided
by actual elapsed time and multiplied by `2*pi/PPR`, averaged over a
sufficient window. Check columns 23–24 for estimate delay during transients.
The firmware geometry is gear=9.6 and radius=0.0315 m; the MATLAB model
uses 9.68 and 0.03 m. Confirm physical values before comparing linear speed
or reflected inertia.

Repeat at multiple voltages and, with a suitable experiment sequence, in
both directions. A repeatable difference can be physical; a difference
that changes with offset correction, voltage or direction suggests the
identification assumptions need revision. Kt=Kb remains an SI assumption,
and J requires physical inertia/load information rather than current logging
alone. Previously tuned gains should be revisited after new identification.
