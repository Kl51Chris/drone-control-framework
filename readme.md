# Drone Control Framework

A modular control framework for Crazyflie-based control experiments. The active
architecture separates reference generation, controller logic, state
acquisition and estimation, hardware-specific command conversion, and reusable
experiment execution.

## Current Capabilities

The active framework includes:

* `BodyRateThrustCommand`, a hardware-independent command containing body-frame
  roll, pitch, and yaw angular-rate references in radians per second plus
  normalized collective thrust in `[0.0, 1.0]`;
* `CrazyflieCommandAdapter`, which converts angular rates from radians per
  second to degrees per second, converts normalized thrust to percentage, and
  calls `send_setpoint_manual(..., rate=True)`;
* `EstimatedState`, for complete state produced by a source-specific estimator,
  and `MeasuredState`, whose fields are optional for direct or external
  measurements;
* the `StateEstimator` interface and `PassthroughStateEstimator`, which currently
  copies `EstimatedState` to the controller-facing `DroneState` and deliberately
  ignores `MeasuredState` (it does not perform sensor fusion);
* `CrazyflieStateProvider`, which acquires position, velocity, attitude, and
  body rates from Crazyflie logs and routes them through a `StateEstimator`;
* `OuterLoopPIDController`, with cascaded world-frame XY position/velocity,
  roll/pitch attitude, altitude, and heading feedback;
* reusable circular and vertical reference generators, plus experiment-specific
  fixed, polarity-check, and slow-trajectory references;
* backend-independent control-loop and generic hardware-runner infrastructure;
* a staged `OuterLoopPIDController` validation experiment with configuration
  checks, command limits, explicit physical-test authorization, and logging;
* automated tests for controller behavior, state estimation, Crazyflie state
  and command conversion, and validation-stage logic.

## Architecture

```text
Reference Generator
        |
        v
     Reference
        |
        v
     Controller <----- DroneState
        |
        v
BodyRateThrustCommand
        |
        v
CrazyflieCommandAdapter
        |
        v
Crazyflie firmware body-rate controller
        |
        v
Motor mixing / motors
```

State follows a separate path:

```text
Crazyflie/onboard estimator -> EstimatedState --\
                                                   -> StateEstimator -> DroneState
External/direct measurement -> MeasuredState -----/
```

The current `PassthroughStateEstimator` uses only the upper input. This boundary
allows a future estimator to combine sources without changing controllers.

## Repository Structure

```text
drone-control-framework/
├── doc/                         technical documentation
├── experiments/                 runnable experiment assembly and configuration
├── legacy/                      historical implementations (not active dependencies)
├── ros2_ws/                     ROS 2 workspace
├── scripts/                     project scripts
├── tools/                       hardware and simulation diagnostics
├── velocity_control_framework/
│   ├── backends/                state providers and command adapters
│   ├── controllers/             active reusable controllers
│   ├── estimators/              state-estimator implementations
│   ├── interfaces/              shared data contracts and protocols
│   ├── logger/                  logging package
│   ├── references/              reusable reference generators
│   ├── runner/                  control-loop and hardware-runner infrastructure
│   └── tests/                   active automated tests
├── requirements-ubuntu-lock.txt
└── requirements-wsl-lock.txt
```

Code under `legacy/` is retained for historical comparison. Active framework
code must not depend on it.

## Interfaces and Units

`DroneState` is the unified controller-facing state:

```text
position       world-frame [x, y, z], m
velocity       world-frame [vx, vy, vz], m/s
roll/pitch/yaw Euler attitude angles, rad
p/q/r          body-frame rates about body x/y/z, rad/s
timestamp      s
```

Body rates `p`, `q`, and `r` are not generally the derivatives of the Euler
angles. `Reference` contains desired world-frame position and velocity, yaw,
yaw rate, and timestamp. The cascaded PID uses XY reference velocity and can
optionally use vertical reference velocity. Heading remains wrapped yaw-error
feedback without yaw-rate feed-forward.

## Active Controller

`OuterLoopPIDController` implements:

```text
DroneState + Reference + dt -> BodyRateThrustCommand
```

It forms bounded world-frame XY acceleration requests from position/velocity
errors, rotates them into the body frame, maps body-X/body-Y acceleration to
bounded pitch/roll references, and closes attitude-P loops into body-rate
commands. It applies altitude PD feedback around equilibrium-thrust
feed-forward and maps wrapped heading error to yaw rate. The experimentally
identified acceleration-to-attitude signs are configurable; the Stage 5
candidate uses `-1` for both axes. Existing staged-validation runners retain an
explicit legacy direct-rate mapping mode. See
[`doc/control-design.md`](doc/control-design.md) for the implemented equations,
numerical PID behavior, coordinate conventions, and Stage 2 experiment law.

## Validation and Development Status

Implemented components include the data interfaces, passthrough estimator,
Crazyflie state and command backends, `OuterLoopPIDController`, reusable
runners and references, and the staged validation program.

Automated tests exercise the PID transformations, saturation and representative
outputs; the passthrough-estimator behavior; Crazyflie conversions; and staged
validation configuration, references, Stage 2 feedback, ramp, and completion.
These tests validate software behavior and do not constitute flight validation.
The calibration toolkit separately records the hardware evidence used for the
cascaded structure and physical mapping signs.

The validation pipeline contains stages 0 through 7, progressing from a
zero-output software pipeline check through polarity, equilibrium-thrust,
stabilization, hover, and slow-trajectory experiments. Physical stages require
operator authorization and reviewed configuration. Calibration demonstrated
bounded custom XY/Z hover, but reliable landing and final production tuning
remain experimental.

Additional state-source integration and sensor fusion remain future work. LQR,
MPC, and DeePC are possible future controller families, but no active
implementation of them exists in `velocity_control_framework/controllers/`.

## Tests

From the repository root:

```bash
pytest velocity_control_framework/tests -v
```

## Production Flight Assembly

`experiments/run_flight.py` is the minimal assembly point for the production
`OuterLoopPIDController`, `stage5_hardware_candidate()` configuration, existing
reference providers, Crazyflie backends, and `UniversalRunner`. Inspect the
fixed-hover selection without connecting to hardware with:

```bash
python3 -m experiments.run_flight
```

One supervised fixed-hover flight can be requested explicitly with `--execute`.
Without that switch the command remains inspection-only and never opens a radio
connection. The flight uses a 0.30 m relative target and
retains the reviewed Stage 5 estimator, envelope, attitude, velocity, timing,
supervisor, and emergency-stop checks. Completion of the descent reference is
followed by `LANDING_CHECK`. At the 15-second maximum duration, success requires
both low estimated height and low vertical speed; this is explicitly an
estimator-qualified landing result, not independent physical-touchdown proof.
An unresolved landing reports `LANDING_FAILED` and uses the emergency-stop and
cleanup path. The existing
circular provider is selectable with `--reference circular`, but powered
circular execution remains interlocked until fixed-hover validation succeeds.

## Documentation

* [`doc/architecture.md`](doc/architecture.md) describes component boundaries,
  runner behavior, staged validation, and implemented safety mechanisms.
* [`doc/control-design.md`](doc/control-design.md) describes the actual control
  equations, units, mappings, and experimental Stage 2 law.
* [`agent.md`](agent.md) contains repository-specific development guidance.
