# Calibration Toolkit

The `calibration/` package is an independent, reproducible toolkit for
measuring and validating Crazyflie state estimation, command conventions, and
custom outer-loop control. It is deliberately separate from the production
control framework: it does not modify `velocity_control_framework/`,
`UniversalRunner`, or the frozen `OuterLoopPIDController`.

The toolkit owns its configurations, flight state machines, telemetry logs,
offline replay, analysis, safety checks, and shutdown handling. A hardware run
always requires explicit per-run confirmation and is never repeated
automatically.

## Start here

- [Stage 5B first-flight procedure](STAGE5B_FIRST_FLIGHT.md)
- [Stage 5C altitude-PD analysis](STAGE5C_ALTITUDE_PD.md)
- [Stage 5D pitch identification](STAGE5D_PITCH_IDENTIFICATION.md)
- [Stage 5D roll identification](STAGE5D_ROLL_IDENTIFICATION.md)
- [Stage 5D XY readiness and acceptance criteria](STAGE5D_XY_READINESS.md)
- [Project inspection notes](INSPECTION.md)

Raw runs are stored under `calibration/runs/` and are intentionally excluded
from Git. Generated replay CSV/SVG/JSON files under `stage5_analysis/` are also
ignored; checked-in Markdown reports and analysis scripts remain the
reproducible record.

## Cascaded PID architecture

`CascadedPIDController` is the unified calibration controller. Its stages are
configuration-gated:

1. Altitude position and measured vertical-velocity feedback, with optional
   integral action and anti-windup (Stage 5C).
2. Roll, pitch, and yaw attitude feedback, retaining the Crazyflie firmware's
   onboard body-rate PID and motor mixer.
3. World-frame XY position/velocity PD, rotated into body coordinates and
   mapped to bounded roll/pitch attitude references (Stage 5D).

Every update returns the existing `BodyRateThrustCommand` units, ordering, and
adapter semantics. XY has no integral term. Offline replay uses recorded states
as exogenous inputs; it is controller-arithmetic replay, not a closed-loop
plant simulation and cannot establish flight stability.

## Important modules

| Module | Purpose |
|---|---|
| `cascaded_pid.py` | Unified configurable altitude, attitude, heading, and XY controller. |
| `cascaded_flight.py` | Hardware runner, fixed profiles, estimator gates, safety checks, logging, landing, and shutdown. |
| `cascaded_replay.py` | Replays Cascaded PID commands through recorded telemetry. |
| `cascaded_analysis.py` | Post-flight metrics, saturation summaries, and response plots. |
| `hardware.py` | Crazyflie/cflib connection, state provider, command adapter, battery snapshot, and emergency stop. |
| `flight_support.py` | Shared battery metadata and run-directory reporting. |
| `pid.py`, `pid_replay.py` | Standalone altitude-PID arithmetic and replay tools. |
| `pitch_response_analysis.py`, `roll_response_analysis.py` | Bipolar physical attitude-to-acceleration identification. |
| `altitude_pd_analysis.py`, `vertical_system_id.py` | Focused altitude response and lightweight vertical-model analysis. |
| `official_hover.py` | Reference experiment using the firmware hover controller; not the custom PID. |
| `state_monitor.py`, `guided_attitude.py`, `motor_free_diagnostic.py` | Readiness, sign, and motor-free diagnostics. |
| `cli.py` | Offline analysis/replay and explicitly authorized hardware command entry point. |

## Configuration model

The flight configuration (for example `default_config.json` or
`stage5b_first_flight_config.json`) contains radio URI, control frequency,
takeoff/hover/landing profile, estimator limits, supervisor limits, and safety
envelopes.

The controller configuration (for example
`stage5c_altitude_pd_pole_candidate_config.json` or
`stage5d_xy_pd_hover_candidate_config.json`) contains controller stage, gains,
thrust bounds, attitude-rate limits, XY bounds, physical mapping signs, and
hardware-validation interlocks. Keeping these files separate makes a replay or
flight profile change auditable without silently changing control law settings.

The Stage 5D XY/Z candidate currently uses altitude `Kp=0.06`, `Kv=0.06`
(zero-reference velocity damping), equilibrium thrust `0.425`, XY `Kp=0.4`,
`Kd=0.8`, bounded acceleration `0.4 m/s²`, and bounded tilt `0.08 rad`.

## Offline tests and analysis

Run the complete calibration suite from the repository root:

```bash
python3 -m pytest calibration/tests
```

Analyze an existing run without reconnecting to hardware:

```bash
python3 -m calibration analyze-cascaded-flight calibration/runs/<run>
python3 -m calibration analyze-pitch-response calibration/runs/<pitch-response-id-run>
python3 -m calibration analyze-roll-response calibration/runs/<roll-response-id-run>
python3 -m calibration analyze-equilibrium calibration/runs/<official-hover-run>
```

Replay controller arithmetic against recorded states:

```bash
python3 -m calibration replay-cascaded-flight \
  calibration/runs/<run> calibration/stage5_analysis/xy-replay \
  --controller-config calibration/stage5d_xy_pd_hover_candidate_config.json
python3 -m calibration replay-pid calibration/runs/<official-hover-run> \
  calibration/stage5_analysis/pid-replay --equilibrium-thrust 0.425
```

Each run contains `metadata.json` and a telemetry CSV. Hardware-flight output
also prints absolute `[RUN]`, `[LOG]`, and `[RESULT]` paths. Battery voltage is
captured once before flight when `pm.vbat` is available; firmware percentage is
recorded only when the firmware exposes `pm.batteryLevel`.

## Physical polarity identification

The pitch and roll identification modes keep XY feedback disabled and apply the
same short fixed timeline: neutral assessment, neutral baseline, +0.020 rad
pulse, neutral recovery, −0.020 rad pulse, neutral recovery, then landing.
Each pulse is admitted only when displacement and velocity envelope margin is
sufficient. Analysis compensates for yaw, searches estimator/attitude delay,
fits local velocity-derived acceleration, and uses a block bootstrap interval.
Missing pulses, saturation, safety events, or an interval containing zero yield
`INCONCLUSIVE`; no sign is changed automatically.

Measured local mappings are:

```text
body-X acceleration / pitch = -8.719 m/s²/rad
body-Y acceleration / roll  = -9.265 m/s²/rad
body_x_acceleration_to_pitch_sign = -1
body_y_acceleration_to_roll_sign  = -1
```

The separate attitude-loop `pitch_command_sign=-1` handles the cflib packet
convention and is not duplicated in the XY mapping.

## Validation status

Validated in hardware:

- Crazyradio communication, supervisor readiness, Flow Deck/state acquisition,
  and estimator freshness/continuity checks.
- Official firmware hover baseline and controlled shutdown behavior.
- Body-rate/thrust command encoding and measured pitch/roll physical polarity.
- Bounded custom Cascaded PID attitude/altitude/XY response with no horizontal
  safety abort in the reviewed XY runs.

Still experimental or incomplete:

- Final altitude/vertical damping and landing behavior; the latest runs remain
  touchdown-confirmation failures.
- Complete custom XY/Z takeoff-hover-land success under changing battery,
  ground-effect, and estimator conditions.
- Transfer of this experimentally supported architecture into the frozen
  production `OuterLoopPIDController`; that integration is a later task.

No completed log alone certifies stability or safety. Review the detailed Stage
documents above before any powered experiment.

## Explicit hardware command

After reviewing the applicable procedure and authorizing exactly one run:

```bash
python3 -m calibration run cascaded-xy \
  --config calibration/stage5b_first_flight_config.json \
  --controller-config calibration/stage5d_xy_pd_hover_candidate_config.json \
  --confirm-flight I_UNDERSTAND_PROPELLERS_MAY_SPIN
```

The command remains subject to the CLI's supervisor, estimator, mapping, and
per-run authorization gates. It does not perform automatic retries or gain
search.
