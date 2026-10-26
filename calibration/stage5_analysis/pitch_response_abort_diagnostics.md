# Pitch-response identification abort diagnostics

Run: `20261026T062705_089178Z_pitch-response-id`

## Root-cause assessment

The horizontal-envelope abort was triggered by an estimator-coordinate
discontinuity, not by physically plausible horizontal travel. The safety action
was correct for the values presented to the runner, but the preflight origin was
no longer a valid continuous reference at the abort.

The vehicle never reached HOVER or any pitch-identification excitation. The
abort occurred in TAKEOFF, 0.3002 s after the first control-loop sample, at
approximately 0.0015 m estimated altitude.

| Point | X (m) | Y (m) | Distance from preflight origin (m) | Horizontal speed (m/s) |
|---|---:|---:|---:|---:|
| Preflight median (25 samples) | 0.781690 | 0.101748 | 0 | not retained |
| First command row | 0.909591 | 0.119554 | 0.129135 | 0.431599 |
| Last telemetry row (0.2802 s) | 0.955029 | 0.126171 | 0.174804 | 0.376744 |
| Abort snapshot (0.3002 s) | 0.236120 | -0.002729 | 0.555484 | 0.065005 |

Between the last row and abort snapshot (0.0200 s), recorded position changed
by (-0.7189, -0.1289) m. Trapezoidal integration of the endpoint velocities
predicts only (+0.00437, +0.00062) m. The position/velocity residual is 0.7348 m.
That change would require an average horizontal speed above 36 m/s and is not a
credible vehicle translation.

Across the logged rows before the abort, position changed 0.0459 m while
velocity integration predicted 0.1161 m (0.0702 m residual). Including the
abort snapshot, position changed 0.6845 m from the first row while integration
predicted 0.1205 m; the vector residual is 0.8050 m. A smaller discontinuity is
also visible at 0.2602 s (0.0445 m residual over 20 ms), before the final reset.

## Timing and preflight gap

The original runner sampled the estimator for 0.5 s, checked only altitude
span and level attitude, and retained only median X/Y/Z/yaw. It did not retain
preflight horizontal velocity or X/Y stability statistics. It then armed and
waited 0.25 s without logging state before obtaining the first command row.

Consequently, the exact instant of the initial 0.129 m origin-to-first-row
change cannot be recovered. It occurred no later than the first command and may
have occurred during the unobserved post-arm interval. The first row's 0.432
m/s estimated speed was already inconsistent with a stationary, ready vehicle.
The later 0.730 m step definitely occurred after commands began: it lies between
the last telemetry row and the abort snapshot.

## Estimator interpretation

No `kalman.resetEstimation` write exists in the calibration flight path, so
there is no evidence that the host deliberately reset the Kalman estimator.
Flow Deck presence was checked, but deck presence does not prove converged or
continuous X/Y state. The Crazyflie provider reports firmware
`stateEstimate.{x,y,vx,vy}`; position and velocity share one translation log
packet. The large position step without matching velocity is therefore
consistent with estimator re-anchoring/convergence, an estimator reset internal
to the firmware, or Flow/estimator initialization behavior. This dataset does
not identify which mechanism occurred, because it contains no estimator
variance, reset-state, quality, or preflight time-series log. Physical motion
is ruled out for the final step; a specific firmware cause is not proven.

## Correction

The cascaded runner now:

- requires the stationary 0.5 s preflight window to remain within 0.05 m of
  its component-wise median and below 0.15 m/s horizontal speed;
- records first/last X/Y, maximum radius, maximum speed, and the readiness
  thresholds in metadata;
- rechecks freshness, horizontal speed, the unchanged origin, the existing
  safety envelope, and position/velocity continuity after arming but before the
  first command;
- compares every in-flight X/Y increment with trapezoidally integrated vx/vy
  and emergency-stops if the residual exceeds 0.10 m; and
- records observed and predicted increments in the safety event.

These are additional estimator-readiness and discontinuity gates. Existing
thrust, attitude, altitude, velocity, displacement, and rate limits are
unchanged. A suspicious post-arm change is rejected; the origin is never
recaptured to hide it.

## Recommendation

**NO-GO for another pitch-response flight until the new pre-arm gates pass and
their metadata is reviewed.** The recorded run provides no pitch-polarity
evidence because excitation never began. After a clean, stationary readiness
record, the prepared experiment can be reconsidered under the existing
per-flight authorization and safety limits. Repeated readiness rejection should
prompt logging of firmware estimator variance/quality and investigation of
Flow Deck optical conditions or estimator initialization before any powered
retry.
