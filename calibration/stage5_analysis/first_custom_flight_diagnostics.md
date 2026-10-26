# First custom Cascaded PID flight diagnostics

Source: `calibration/runs/20261026T053025_672531Z_cascaded-5b`. Analysis uses all 207 logged command rows; the safety snapshot at 0.50326 m displacement is discussed separately because it was intentionally persisted before shutdown rather than written as a normal command row.

## TAKEOFF versus HOVER

Error is `reference - actual`; negative error means the vehicle is above reference.

| phase | samples | duration (s) | altitude RMSE (m) | mean error (m) | max abs error (m) | mean vz (m/s) | actual z range (m) | reference z range (m) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| TAKEOFF | 100 | 2.000 | 0.09145 | -0.06649 | 0.13482 | +0.15861 | -0.0001–0.3194 | -0.0000–0.1980 |
| HOVER | 107 | 2.140 | 0.07606 | -0.04876 | 0.12168 | -0.08276 | 0.1465–0.3216 | 0.2000–0.2000 |

Takeoff had the larger RMSE (0.09145 m versus 0.07606 m) and maximum error. The takeoff ramp lagged initially, then the vehicle overshot: actual altitude reached 0.3194 m while the reference ended near 0.1980 m. HOVER began during this overshoot, so its aggregate negative mean error (-0.04876 m) does not represent a settled offset.

![Altitude response](first_custom_flight_altitude.svg)

## Hover time windows

| seconds after HOVER start | samples | mean z (m) | mean error (m) | mean vz (m/s) | fitted z slope (m/s) | mean thrust |
|---|---:|---:|---:|---:|---:|---:|
| 0.0–0.5 | 26 | 0.3159 | -0.1159 | -0.0255 | -0.0274 | 0.4134 |
| 0.5–1.0 | 25 | 0.2874 | -0.0875 | -0.0794 | -0.0807 | 0.4163 |
| 1.0–1.5 | 25 | 0.2360 | -0.0361 | -0.1210 | -0.1113 | 0.4214 |
| 1.5–2.0 | 25 | 0.1763 | +0.0237 | -0.1122 | -0.1138 | 0.4274 |
| 2.0–2.5 | 6 | 0.1509 | +0.0491 | -0.0629 | -0.0664 | 0.4299 |

The 2.14 s hover record does not establish steady state. Altitude descended from the takeoff overshoot throughout nearly the whole interval. Mean error changed from -0.1159 m in the first 0.5 s to +0.0491 m in the final partial window; thus altitude did not remain consistently below reference. At termination z was 0.1465 m versus the 0.2000 m reference, and vz was -0.0669 m/s: it was below reference and still descending. There is no defensible steady-state offset estimate from this interval.

## Error, vertical velocity, thrust, and P contribution

Over the entire run, error–vz correlation was -0.191; during HOVER it was -0.635. Error–thrust correlation was +1.000000. The last value is mechanically expected, not independent evidence: with unsaturated P-only control, `thrust = 0.425 + 0.1*error`, so altitude P and thrust are exact affine transforms of error.

Observations: takeoff showed lag followed by substantial overshoot; hover showed a continuing descent through the reference; no altitude or thrust command saturated. Hypotheses consistent with those observations include missing vertical-velocity damping and transient reference-tracking lag. An equilibrium-thrust mismatch may also contribute, but this flight does not isolate it. The data do not identify insufficient P gain: increasing P could reduce static error but could also worsen the observed overshoot without damping. Causal gain selection is not justified from this single short trajectory.

## Horizontal drift

The last logged displacement vector from the captured origin was (-0.2989, -0.4004) m, magnitude 0.4996 m at -126.7 degrees. The subsequent safety snapshot was 0.50326 m. Mean velocity vectors were TAKEOFF (-0.0388, -0.0653) m/s and HOVER (-0.1102, -0.1244) m/s; whole-run mean was (-0.0757, -0.0959) m/s.

The direct displacement was 0.714 of the sampled XY path length, and 96.6% of velocity samples projected in the final displacement direction. Drift was therefore predominantly toward negative X and negative Y, though not perfectly straight. Mean speed rose from 0.0892 m/s in TAKEOFF to 0.1782 m/s in HOVER. This is consistent with XY feedback being disabled: no control term attempted to return position or damp horizontal velocity. It does not establish the source of the drift; trim, small attitude bias, takeoff transients, floor/Flow Deck effects, and estimator behavior remain candidates.

![XY response](first_custom_flight_xy.svg)

## Attitude and estimated horizontal acceleration

Acceleration was estimated from local least-squares slopes of vx/vy over centered 0.4 s windows (typically about 20 samples), then rotated into the measured-yaw body frame. This suppresses pointwise differentiation noise but also smooths and phase-shifts short dynamics. Zero-lag correlations were roll versus body-Y acceleration -0.705, and pitch versus body-X acceleration -0.408. Varying the total regression window from 0.3 to 0.8 s kept these correlations in approximately [-0.78, -0.63] and [-0.45, -0.32], respectively, so their signs are not a single-window artifact.

The roll/body-Y negative sign agrees with the controller mapping `roll_ref = -atan2(ay_body,g)`. The pitch/body-X result is opposite the controller's abstract `pitch_ref = atan2(ax_body,g)` mapping, but recorded pitch uses the Crazyflie legacy pitch convention and the command path performs a pitch sign conversion. This short closed-loop record cannot cleanly separate that coordinate convention, response lag, and coupled motion; it is not sufficient to identify a physical pitch-to-acceleration coefficient.

Maximum roll and pitch were only 0.0246 and 0.0680 rad, so a small persistent tilt or trim error could plausibly explain part of the accumulated drift. The trajectory cannot separate that effect from Flow Deck noise/filtering, aerodynamic transients, estimator lag, or yaw-coordinate coupling. Raw numerical acceleration is not used.

![Attitude and acceleration](first_custom_flight_attitude_acceleration.svg)

## Recommendation

**B — enable bounded XY position/velocity feedback first.** The direct reason the experiment ended was the horizontal boundary, not attitude, altitude, or command saturation. Bounded attitude response and zero rate saturation suggest enough margin to test a conservative XY loop, while uncontrolled mean horizontal velocity approximately doubled from TAKEOFF to HOVER and prevented collecting a longer altitude record. Adding XY control is therefore the most direct way to extend diagnostic time for subsequent altitude work.

Altitude dynamics still merit the next priority after horizontal containment: the overshoot and continuing descent strongly motivate vertical-velocity damping before increasing altitude P. This recommendation does not authorize gains or a flight and does not claim XY control will remove all drift.

## Limitations

- One 4.12 s run, only 2.14 s of HOVER, no landing, and no repeated trials.
- The HOVER phase began in a takeoff overshoot and never reached demonstrated equilibrium.
- P contribution and thrust are algebraically coupled, so their correlation with error is not causal evidence.
- Flow Deck estimates are filtered, serially correlated, and may degrade with surface, height, or motion.
- Local-regression acceleration is smoother than raw differentiation but remains an estimator-derived approximation.
- XY control was disabled, so this dataset cannot predict closed-loop XY stability or select XY gains.
