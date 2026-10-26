# Flight Logging

Production flight records are written under the project-level `log/` directory
by `velocity_control_framework.logger.FlightLogger`. The logger is independent
of controller implementations and is attached to `UniversalRunner` by an
experiment runner.

## Initializing a run

An experiment supplies a human-readable parameter-set name and complete run
provenance before connecting the logger to the runner:

```python
from dataclasses import asdict
from velocity_control_framework.logger import FlightLogger, RunConfiguration

logger = FlightLogger(RunConfiguration(
    experiment_name="fixed-hover",
    reference_type="hover",
    reference_configuration={"height_m": 0.3},
    controller_type="outer-loop-pid",
    parameter_set="stage5-hardware-candidate",
    controller_configuration=asdict(controller.config),
    target_control_frequency_hz=50.0,
    hardware="Crazyflie",
    state_estimation_source="Crazyflie stateEstimate via passthrough",
))

runner = UniversalRunner(..., run_logger=logger)
```

`parameter_set` is a stable, human-readable experiment identifier such as
`stage5-hardware-candidate` or `tune-v3`; it is not inferred from gain values.
The production `experiments.run_flight` entry point performs this setup.

Directories use
`YYYYMMDD_HHMMSS_reference_controller_paramset_experiment`. If that path
already exists, the logger appends part of the UUID run identifier rather than
overwriting it. `metadata.json` contains the full UUID, timezone-aware start
and end times, Git commit when available, configuration, source provenance,
and terminal status.

## Files and schemas

- `flight.csv` has one row per controller update: monotonic execution time,
  actual `dt`, phase, reference, the `DroneState` consumed by the controller,
  command, safety status, source-sequence associations, and optional controller
  diagnostics.
- `onboard_state.csv` stores asynchronous Crazyflie `stateEstimate` samples.
  These are onboard estimator outputs, not raw IMU measurements.
- `external_state.csv` stores optional external-provider measurements. It is
  header-only when no source is connected; no onboard or estimated values are
  substituted.
- `events.csv` stores lifecycle and phase events with JSON details.
- `metadata.json` stores configuration and provenance and transitions through
  `initialized`, `running`, and a terminal status such as `completed`,
  `aborted`, or `failed`.

Empty CSV cells and a false availability flag mean a measurement was not
available. They do not mean zero. Source streams are recorded on arrival and
are not resampled or artificially synchronized. A flight row's
`onboard_sequence` and `external_sequence` identify the latest source samples
available for that controller step.

## Coordinates, units, and timestamps

Position and velocity are world-frame `[x, y, z]` in metres and metres per
second. Roll, pitch, and yaw are radians. `p`, `q`, and `r` are body-frame
angular rates in radians per second. Commands are body rates in radians per
second and normalized collective thrust in `[0, 1]`.

`source_timestamp_s` preserves the source clock when exposed. For Crazyflie
log packets this is the firmware log timestamp converted from milliseconds to
seconds. `reception_monotonic_s` is host monotonic arrival time. The controller
state timestamp remains the host monotonic assembly time used by existing
safety checks. These clocks must not be compared without clock alignment.
Raw IMU data is not present because the active telemetry interface does not
expose it.

The estimator boundary remains `EstimatedState + MeasuredState -> DroneState`.
The active passthrough estimator uses onboard `EstimatedState` and receives an
empty `MeasuredState`. A future Motive provider can call
`record_external_state(measurement, source_timestamp_s=...,
reception_monotonic_s=...)` from its arrival callback, then expose that
sequence in its state-provider logging context. Logging does not require a
Motive connection.

## Controller-specific diagnostics

A flight program may implement `get_logging_diagnostics()` and return a small
JSON-serializable mapping. The runner stores it in
`controller_diagnostics_json`; common columns never change between PID, MPC,
and DeePC. Diagnostic collection must be side-effect free and must not perform
disk or network I/O.

All CSV writes are queued to a background writer. Normal completion,
interrupts, and exceptions drain the queue and finalize metadata after command
and state-provider cleanup, preserving partial logs where possible.
