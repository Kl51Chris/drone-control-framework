# Drone Control Framework

A modular control framework for Crazyflie-based drone experiments, with support for interchangeable controllers, trajectory references, hardware backends, and reusable flight infrastructure.

The project is intended to separate control logic from hardware-specific implementation so that controllers such as PID, LQR, and MPC can be developed and tested within a consistent software architecture.

## Current Capabilities

The repository currently includes:

* Crazyflie hardware integration through `cflib`
* Flow Deck / ZRanger-based state feedback
* Circular and vertical flight references
* Hardware-independent controller interfaces
* Crazyflie-specific command and state backends
* Reusable control-loop and hardware-runner infrastructure
* Hardware diagnostic and test utilities
* Legacy implementations retained for reference

## High-Level Architecture

The main control flow is:

```text
Reference Generator
        |
        v
     Reference
        |
        v
DroneState --> Controller
                  |
                  v
         BodyRateThrustCommand
                  |
                  v
            CommandAdapter
                  |
                  v
              Hardware
```

State feedback flows from the hardware through a state provider:

```text
Crazyflie/onboard estimator -> EstimatedState --\
                                                   -> StateEstimator -> DroneState
External/direct measurement -> MeasuredState -----/
```

The framework is designed so that:

* controllers do not directly communicate with Crazyflie hardware
* hardware-specific unit conversion remains inside backend adapters
* reference generation remains independent of controller implementation
* experiments configure and run reusable framework components
* legacy code is preserved separately from active framework development

## Repository Structure

```text
drone-control-framework/
├── experiments/
├── legacy/
├── scripts/
├── tools/
├── velocity_control_framework/
├── requirements-ubuntu-lock.txt
├── requirements-wsl-lock.txt
└── README.md
```

### `velocity_control_framework/`

Reusable control-system components.

```text
velocity_control_framework/
├── backends/
├── controllers/
├── interfaces/
├── logger/
├── references/
├── runner/
└── tests/
```

* `interfaces/`

  * shared data structures and controller contracts
  * includes `DroneState`, `Reference`, and `BodyRateThrustCommand`

* `controllers/`

  * location for reusable active control implementations
  * obsolete controllers are retained under `legacy/`

* `references/`

  * trajectory and reference generators

* `backends/`

  * hardware-facing interfaces
  * Crazyflie command conversion
  * Crazyflie state acquisition

* `runner/`

  * reusable control-loop and hardware-execution infrastructure

* `tests/`

  * framework-level automated tests

### `experiments/`

Runnable flight experiments and experiment-specific configuration.

Experimental scripts should use reusable framework components instead of implementing new controller logic directly.

### `tools/`

Hardware, Crazyflie, and simulation diagnostics.

Examples include:

* Crazyradio scanning
* Crazyflie connection checks
* firmware parameter inspection
* hardware command tests
* MuJoCo utilities

### `legacy/`

Older implementations retained for reference.

New framework code should not depend on modules under `legacy/`.

## Control Interfaces

The framework currently uses the following main abstractions.

### Drone State

`EstimatedState` represents source-specific state from the Crazyflie/onboard
estimator. `MeasuredState` represents optional direct or external measurements
and allows fields to be unavailable. `StateEstimator` is the boundary that
selects or combines those sources into `DroneState`, the unified state used by
controllers.

The initial `PassthroughStateEstimator` intentionally constructs `DroneState`
only from `EstimatedState`; it accepts but ignores `MeasuredState`. This is
temporary plumbing for future measurement integration, not sensor fusion.

It currently contains:

```text
position [x, y, z]
velocity [vx, vy, vz]
roll
pitch
yaw
timestamp
p
q
r
```

Positions are represented in meters, velocities in meters per second, attitude
angles in radians, and body angular rates in radians per second. `p`, `q`, and
`r` are rotations about the body x, y, and z axes; they are not generally equal
to roll, pitch, and yaw Euler-angle derivatives.

### Reference

`Reference` represents the desired motion or trajectory.

It currently contains:

```text
position
velocity
yaw
yaw_rate
timestamp
```

### Controller Output

Controllers produce a `BodyRateThrustCommand`:

```text
roll_rate
pitch_rate
yaw_rate
thrust
```

Framework-side units are:

```text
roll_rate   body x-axis angular-rate reference, rad/s
pitch_rate  body y-axis angular-rate reference, rad/s
yaw_rate    body z-axis angular-rate reference, rad/s
thrust      normalized collective thrust [0.0, 1.0]
```

The angular-rate fields are body-axis references, not Euler attitude angles or
Euler-angle derivatives. The Crazyflie backend converts these values into
Crazyflie-specific command units and sends a manual setpoint with `rate=True`.

## Controllers

The active package currently defines the controller contract but does not
bundle a controller implementation. Controllers must return
`BodyRateThrustCommand` and remain independent of backend-specific APIs and
units. Earlier velocity/altitude controllers and their runners are retained
under `legacy/` for historical reference.

## Running Tests

From the repository root:

```bash
pytest velocity_control_framework/tests -v
```

Framework changes should keep these tests passing.

## Development Status

The repository is currently being reorganized into a clearer separation between:

* reusable framework code
* flight experiments
* diagnostics and hardware tools
* legacy implementations

Current development areas include:

* improving repository structure
* refining controller and runner interfaces
* yaw / yaw-rate control development
* improved state-feedback integration
* future motion-capture integration

## Documentation

More detailed project documentation will be added under:

```text
docs/
```

Planned documentation includes:

```text
docs/architecture.md
docs/control-design.md
```

These documents will describe the software architecture, interface boundaries, coordinate conventions, control equations, and controller design decisions in greater detail.
