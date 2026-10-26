# Minimal pitch-to-body-X physical response identification

## Decision

**Software GO to conduct exactly one explicitly authorized pitch-response
identification flight; NO-GO for XY containment flight.** The identification mode
keeps XY position control disabled and exists solely to resolve the remaining
physical mapping sign. `horizontal_mapping_hardware_validated` remains false and
the `cascaded-xy` CLI continues to refuse hardware connection.

No flight was initiated while preparing this procedure. No altitude/attitude
gain, thrust bound, body-rate bound, or safety limit was changed.

## Exact command and configurations

```bash
python -m calibration run pitch-response-id \
  --config calibration/pitch_identification_flight_config.json \
  --controller-config calibration/stage5c_altitude_pd_pole_candidate_config.json \
  --confirm-flight I_UNDERSTAND_PROPELLERS_MAY_SPIN
```

The controller configuration is the Stage 5C pole-placement candidate: altitude
Kp/Kv 0.06/0.06 with zero-reference velocity damping and no integral feedback,
roll/pitch/yaw gains 4/4/2,
equilibrium/min/max thrust 0.425/0.380/0.450, and 0.35 rad/s rate limits. XY is
disabled. The flight configuration preserves every existing safety threshold.
The identification timeline uses a 0.10 m target with a 1.0 s takeoff and starts
the existing controlled landing at 2.5 s.

## Readiness gate and excitation sequence

No near-stationary XY readiness condition is imposed because XY feedback is
disabled. The fixed sequence is:

| identification phase | pitch reference | duration |
|---|---:|---:|
| TAKEOFF | 0 | 0–1.00 s |
| SAFETY_ASSESSMENT | 0 | 1.00–1.20 s |
| NEUTRAL_BEFORE | 0 | 1.20–1.40 s |
| PITCH_POSITIVE | +0.020 rad | 1.40–1.65 s |
| RECOVERY_POSITIVE | 0 | 1.65–1.90 s |
| PITCH_NEGATIVE | -0.020 rad | 1.90–2.15 s |
| RECOVERY_NEGATIVE | 0 | 2.15–2.50 s |
| LANDING | 0 | from 2.50 s |

Immediately before each pulse, the runner checks current horizontal
displacement and speed against the unchanged hard envelope. It reserves 0.05 m
beyond constant-velocity projected motion through the planned pulse/recovery
and requires 0.10 m/s speed margin. A failed margin check skips only that pulse,
keeps pitch neutral, records the reason, and makes the result INCONCLUSIVE.
Actual hard-envelope or estimator-continuity violations retain emergency stop.

The bipolar design distinguishes sign from initial velocity, trim, and baseline
acceleration better than a single pulse. The 0.020 rad (1.15 degree) reference is
well below the unchanged 0.45 rad flight tilt boundary. The existing attitude
loop and 0.35 rad/s body-rate bounds remain authoritative. After the sequence,
the reference stays neutral until the normal landing profile.

## Logging and shutdown

The existing persistent flight log adds identification phase, excitation pitch
reference, and timeline state. Metadata records both pulse-margin checks,
performed phases, and whether both excitations occurred. The telemetry already records monotonic time/dt, full XYZ
position and velocity, yaw, measured pitch, body rate q, desired pitch after the
controller, raw/limited pitch-rate command, thrust, saturation flags, supervisor
state, limits, and abort snapshot. Rows are flushed before command transmission.

All established state-age, deadline, tilt, altitude, vertical-speed, horizontal
displacement/speed, thrust, rate, supervisor emergency-stop, repeated stop, and
disarm paths remain active. Ctrl+C still terminates lift rather than guaranteeing
a controlled landing, and no software shutdown can guarantee motor stop during
radio loss.

## Analysis

After the single flight:

```bash
python -m calibration analyze-pitch-response calibration/runs/<pitch-response-id-run>
```

The analysis writes `pitch_response_analysis.json` and separates:

1. **command-to-attitude polarity:** scans 0–0.30 s lag between pitch reference
   and measured legacy pitch, then compares positive and negative pulse pitch
   against the neutral baseline and reports q;
2. **attitude-to-acceleration polarity:** estimates world acceleration with
   centered 0.30 s local linear fits to vx/vy, rotates it into body axes using
   measured yaw, scans 0–0.30 s pitch-to-acceleration lag, and regresses body-X
   acceleration on measured pitch;
3. **uncertainty:** reports a deterministic 95% moving-block-bootstrap interval
   for the physical slope, preserving short-range serial correlation better than
   independent-sample error bars.

Acceleration is derived from velocity slopes, so constant initial velocity does
not bias the slope; initial world/body velocity is nevertheless reported. Local
regression and lag scanning reduce but do not eliminate Flow Deck filtering,
estimator delay, vibration, coupling, or ground-effect limitations.

## Objective acceptance criteria

The automatic result is `PASS` only when all of these hold:

- at least 10 samples exist in the neutral, both pulse, and both recovery phases;
- positive reference changes measured legacy pitch by at least +0.005 rad from
  neutral and negative reference changes it by at least -0.005 rad;
- the 95% block-bootstrap interval for body-X acceleration per measured-pitch
  radian lies wholly above or wholly below zero;
- pitch-rate saturation is <=5% of identification samples;
- no safety event occurred.

The result is `INCONCLUSIVE` if a phase is incomplete, pulse responses do not
have both expected command-to-attitude signs, the physical-slope interval crosses
zero, positive/negative responses conflict on review, the best lag reaches the
search boundary, a safety event occurs, or estimator quality is questionable.
An inconclusive result must not change any sign or interlock.

A reported physical sign is evidence for human review, not an automatic config
edit. Review must compare reference, measured pitch, q, body-X acceleration,
yaw, saturation, and both pulse directions. Only after that review may a human
set `body_x_acceleration_to_pitch_sign` and
`horizontal_mapping_hardware_validated`; software never changes either.
