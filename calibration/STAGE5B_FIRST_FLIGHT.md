# Stage 5B first custom-controller flight readiness

## Decision and command

The first explicitly authorized experiment has now been completed. It produced
bounded attitude and altitude-control evidence and then aborted at 0.50326 m
horizontal displacement during HOVER, before completing the profile. See
`STAGE5B_FIRST_FLIGHT_RESULT.md`. This command remains documentation of the
reproducible procedure, not authorization for another flight.

Exact command:

```bash
python -m calibration run cascaded-5b \
  --config calibration/stage5b_first_flight_config.json \
  --controller-config calibration/stage5b_offline_config.json \
  --confirm-flight I_UNDERSTAND_PROPELLERS_MAY_SPIN
```

The literal confirmation is required for each new process. The command starts
one profile and has no repeat or gain search. It uses `CascadedPIDController`,
creates `BodyRateThrustCommand`, and sends it through `CrazyflieHardware`; it
does not call the official hover controller.

## Exact configuration

Controller settings are exactly those in `stage5b_offline_config.json`:

- altitude P=0.10; vertical-velocity and integral feedback disabled;
- equilibrium/min/max normalized thrust 0.425/0.380/0.450;
- level roll/pitch and captured-initial-heading references;
- roll/pitch/yaw gains 4.0/4.0/2.0;
- all body-rate limits 0.35 rad/s;
- XY control disabled.

The controller file retains `powered_attitude_validated=false` as accurate
provenance. `hardware_flight_authorized=false` is also retained because durable
configuration is not flight authorization; the mandatory CLI literal records
the operator's per-process authorization.

The flight configuration is `stage5b_first_flight_config.json`. Safety limits
are unchanged from the default: state age 0.15 s, loop dt 0.10 s, tilt 0.45 rad,
absolute estimator altitude 1.5 m, vertical speed 1.5 m/s, horizontal displacement
0.50 m from the preflight origin, horizontal speed 0.75 m/s, and maximum thrust
0.45. The effective controller minimum is 0.38. Rate limits are 0.35 rad/s.

## Profile and timing

After a 0.5 s stationary estimator check and arming confirmation:

| phase | reference | nominal duration |
|---|---|---:|
| TAKEOFF | ramp from measured ground Z to +0.20 m at 0.10 m/s | 2.0 s |
| HOVER | hold +0.20 m | 3.0 s |
| LANDING | ramp to measured ground Z at 0.08 m/s | 2.5 s |
| SETTLE | command measured ground Z | 0.5 s |
| LANDING_CONFIRM | continue ground-Z command; require relative height <=0.06 m and abs(vz)<=0.08 m/s continuously | >=0.5 s |

Nominal commanded duration is about 8.5 s. If touchdown is not confirmed, the
runner continues the ground reference for at most 3 s, then treats the condition
as hard safety and requests supervisor emergency stop. P-only altitude control
may not unload thrust enough to land smoothly; this is a known first-flight risk.

## Logging and shutdown

`flight_telemetry.csv` is created and its header flushed before connection.
Every control row is flushed before its corresponding command send and contains
all references, XYZ position/velocity, attitude, body rates p/q/r, controller
errors and contributions, desired attitudes, raw/limited commands, saturation
flags, limits, state age, loop dt, and supervisor state. Thus a send failure
retains the intended command, though it cannot prove that the radio transmitted
it.

Hard safety ordering is: request supervisor emergency stop, attempt repeated
stop setpoints, attempt disarm, then persist the triggering snapshot and finalize
metadata. This prevents diagnostic disk I/O from delaying emergency shutdown.
A zero-row abort still retains the latest state in `metadata.json`.

Ctrl+C requests repeated stop setpoints and disarm and finalizes the log as
`interrupted`; it does not attempt a controlled landing. Unexpected exceptions
also request stop/disarm and preserve the exception plus available rows. During
radio loss all shutdown calls are best-effort: **Ctrl+C or Python cleanup cannot
guarantee physical motor shutdown when communication is unavailable.** The
observer must retain physical power-removal access.

Postflight analysis:

```bash
python -m calibration analyze-cascaded-flight calibration/runs/<cascaded-5b-run>
```

It supports complete, interrupted, hard-aborted, exception-aborted, and zero-row
runs; writes `flight_analysis.json` and, when samples exist, `flight_response.svg`.

## Operator review immediately before execution

- Use a cleared, isolated area, level textured surface, charged battery, secure
  Flow Deck and propellers, an observer, eye protection, and reachable battery
  disconnect. No other commander may be connected.
- Review both exact JSON files and confirm the URI and unchanged bounds.
- Confirm protocol >=12, Flow Deck detection, emergency-stop availability,
  settled fresh state, level vehicle, and safe supervisor state.
- Expect failed takeoff, abrupt landing, attitude divergence, or a minor vehicle
  crash. Personnel must remain outside the bounded test area.
- Abort for unexpected sound, trajectory, attitude, observer concern, or loss of
  telemetry. Do not start a second run until the first artifact is analyzed.

Known risks still include repeatability of powered attitude dynamics, adequacy and
battery dependence of 0.425 equilibrium thrust, P-only landing authority, yaw
heading transients, radio-loss shutdown, estimator behavior during impact, and
the physical effectiveness of supervisor emergency stop on this firmware build.
