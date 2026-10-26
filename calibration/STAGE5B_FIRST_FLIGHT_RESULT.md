# Stage 5B first custom-controller hardware result

## Outcome

The first powered `CascadedPIDController` experiment was executed on 2026-10-26
using `stage5b_first_flight_config.json` and `stage5b_offline_config.json`. It
successfully produced initial powered attitude and altitude-control evidence,
but did **not** complete the full takeoff-hover-land profile.

Run identifier: `20261026T053025_672531Z_cascaded-5b`.

The run entered HOVER and recorded 207 control samples over 4.1201 s. Horizontal
position was intentionally not controlled in Stage 5B. The configured 0.50 m
horizontal-displacement boundary triggered at 0.50326 m. The runner classified
this as hard safety, requested supervisor emergency stop, sent stop setpoints,
and disarmed. All three shutdown operations reported success in software.

## Measured summary

- Termination: `hard safety: horizontal envelope exceeded`
- Abort displacement: 0.50326 m
- Horizontal speed at abort: 0.17463 m/s
- Maximum logged horizontal speed: 0.19847 m/s
- Altitude at abort: 0.14516 m
- Maximum absolute vertical speed: 0.35682 m/s
- Altitude RMSE over recorded profile: 0.08385 m
- Maximum absolute altitude error: 0.13482 m
- Maximum roll / pitch deviation: 0.02459 / 0.06804 rad
- Maximum heading error: 0.05772 rad
- Thrust min / mean / max: 0.41152 / 0.41927 / 0.43034
- Altitude, roll-rate, pitch-rate, and yaw-rate saturation samples: zero

Immediately before abort, roll and pitch remained close to level and commands
were bounded. The final logged command was roll rate 0.01682 rad/s, pitch rate
-0.00047 rad/s, yaw rate 0.10425 rad/s, and normalized thrust 0.43034. The safety
snapshot captured the first state beyond the boundary even though the last normal
telemetry row was at 0.49961 m displacement.

## Interpretation

This is successful initial experimental evidence for powered attitude response,
altitude command generation, persistent logging, boundary detection, and the
software shutdown sequence. It is not a successful full-profile flight and does
not validate hover performance, landing, radio-loss shutdown, or XY control.

The bounded roll/pitch response is encouraging, but one aborted run is not enough
to mark the controller hardware-validated. The displacement abort is expected
for a controller with XY feedback disabled and demonstrates that the configured
horizontal safety boundary operated. No automatic gain or safety-limit change is
justified by this result.

Raw run files remain under `calibration/runs/` and are intentionally excluded
from version control. This durable summary contains the evidence needed to
interpret the committed Stage 5B implementation without committing generated
telemetry or analysis artifacts.
