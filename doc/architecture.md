# Software Architecture

## 1. Purpose

This document defines the software architecture of the Drone Control Framework.

The primary design goal is to separate:

* trajectory generation
* control algorithms
* vehicle state estimation
* hardware-specific command conversion
* execution and experiment configuration

This separation allows controllers, references, and hardware backends to evolve independently without requiring the entire flight stack to be rewritten.

The framework is intended to support multiple control approaches, including PID, LQR, MPC, and other future controllers, while preserving a common software interface.

---

## 2. High-Level Control Architecture

The main control data flow is:

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
         AttitudeThrustCommand
                  |
                  v
            CommandAdapter
                  |
                  v
              Hardware
```

State feedback flows in the opposite direction:

```text
Hardware
   |
   v
StateProvider
   |
   v
DroneState
```

The runner coordinates these components and determines when each update occurs.

A complete control-loop view is therefore:

```text
                   Reference Generator
                           |
                           v
                       Reference
                           |
                           |
Hardware --> StateProvider --> DroneState
                           |       |
                           |       v
                           +--> Controller
                                   |
                                   v
                          AttitudeThrustCommand
                                   |
                                   v
                            CommandAdapter
                                   |
                                   v
                               Hardware
```

---

## 3. Repository-Level Structure

The repository is divided into four main categories:

```text
drone-control-framework/
├── velocity_control_framework/
├── experiments/
├── tools/
├── legacy/
└── docs/
```

Each directory has a distinct responsibility.

### `velocity_control_framework/`

Contains reusable control-system infrastructure.

Code in this directory should represent components that can be reused across multiple experiments.

The framework is currently divided into:

```text
velocity_control_framework/
├── interfaces/
├── controllers/
├── references/
├── backends/
├── runner/
├── logger/
└── tests/
```

### `experiments/`

Contains runnable flight experiments and experiment-specific configuration.

Experiments may select:

* a controller
* controller parameters
* a reference generator
* a hardware backend
* experiment duration
* flight sequence configuration

Experiments should assemble reusable framework components rather than implement reusable controller logic directly.

### `tools/`

Contains diagnostic, hardware-test, inspection, and simulation utilities.

Typical examples include:

* Crazyradio scanning
* Crazyflie connection checks
* firmware parameter inspection
* motor and thrust tests
* simulation inspection utilities

Tools are not part of the normal control-loop architecture.

### `legacy/`

Contains older implementations preserved for historical reference.

New framework code must not depend on modules under `legacy/`.

Legacy code may be inspected when useful, but it should not become part of the active dependency graph.

---

## 4. Interfaces

The `interfaces/` package defines the data contracts shared between framework components.

These interfaces form the most important architectural boundary in the project.

Current core interfaces are:

```text
DroneState
Reference
AttitudeThrustCommand
Controller
```

---

## 5. DroneState

`DroneState` represents the vehicle state available to controllers.

The current state representation includes:

```text
position
velocity
roll
pitch
yaw
timestamp
```

The public framework convention is:

```text
position    world frame, meters
velocity    world frame, meters/second

roll        radians
pitch       radians
yaw         radians

timestamp   seconds
```

Hardware-specific measurement formats must be converted into this representation before being exposed to controllers.

Controllers should not need to know whether the state originated from:

* Crazyflie onboard estimation
* Flow Deck
* ZRanger
* motion capture
* simulation
* another future state source

The state provider is responsible for performing this conversion.

---

## 6. Reference

`Reference` represents the desired vehicle motion.

The current reference interface contains:

```text
position
velocity
yaw
yaw_rate
timestamp
```

A reference generator determines how these values evolve over time.

Examples include:

* hover
* vertical takeoff and landing
* circular velocity trajectories
* future position trajectories

Reference generators should describe desired motion only.

They must not depend on a specific controller implementation.

For example, a circular reference should not need to know whether the active controller is PID, LQR, or MPC.

---

## 7. Controller

Controllers implement the common controller interface:

```text
DroneState
+
Reference
+
dt
    |
    v
AttitudeThrustCommand
```

Conceptually:

```text
Controller.update(
    state,
    reference,
    dt
)
```

returns an `AttitudeThrustCommand`.

Controllers are pure control components.

A controller may:

* compute feedback errors
* maintain internal controller states
* integrate errors
* solve optimization problems
* apply controller-specific limits

A controller must not:

* directly call Crazyflie APIs
* directly access Crazyradio
* open hardware connections
* depend on experiment scripts
* depend on legacy implementations

Hardware communication belongs outside the controller layer.

This boundary allows the same controller logic to be used with different hardware or simulation backends.

---

## 8. AttitudeThrustCommand

`AttitudeThrustCommand` is the hardware-independent control output produced by controllers.

The current command representation is:

```text
roll
pitch
yaw_rate
thrust
```

Framework-side units are:

```text
roll        radians
pitch       radians
yaw_rate    radians/second
thrust      normalized [0, 1]
```

These units are deliberately independent of the Crazyflie API.

Controllers should always produce commands using framework units.

They must not perform Crazyflie-specific conversion.

---

## 9. Backends

The `backends/` package forms the boundary between the reusable control framework and hardware-specific implementation.

There are currently two major backend responsibilities:

```text
StateProvider
CommandAdapter
```

---

## 10. StateProvider

A state provider obtains vehicle measurements and converts them into `DroneState`.

Conceptually:

```text
Hardware measurements
        |
        v
   StateProvider
        |
        v
     DroneState
```

Hardware-specific details belong here.

Examples include:

* Crazyflie log variable names
* sensor availability
* estimator output conventions
* unit conversion
* future motion-capture integration

Controllers should consume only `DroneState`, not raw hardware measurements.

---

## 11. CommandAdapter

A command adapter converts `AttitudeThrustCommand` into hardware-specific commands.

For the Crazyflie backend, the current conversion includes:

```text
roll:
    radians -> degrees

pitch:
    radians -> degrees

yaw_rate:
    radians/second -> degrees/second

thrust:
    normalized [0, 1] -> uint16
```

The adapter then sends the converted command through the Crazyflie commander interface.

Any hardware-specific:

* unit conversion
* sign convention
* command encoding
* API call

belongs in the backend adapter.

These details must not leak into controllers.

---

## 12. References

The `references/` package contains reusable reference generators.

Reference generators determine desired motion as a function of time or flight state.

Conceptually:

```text
time / experiment state
        |
        v
Reference Generator
        |
        v
     Reference
```

Reference generators should remain controller-independent.

A controller may decide how to track a reference, but the reference generator should only specify what motion is desired.

---

## 13. Runner Layer

The runner layer coordinates execution of the control system.

Its responsibilities include:

* obtaining the current state
* obtaining or updating the current reference
* computing controller update timing
* calling the controller
* sending the resulting command
* controlling flight-loop sequencing
* stopping safely when execution ends

Conceptually:

```text
while experiment is running:

    state = state_provider.get_state()

    reference = reference_generator(...)

    command = controller.update(
        state,
        reference,
        dt,
    )

    command_adapter.send(command)
```

The runner is orchestration code.

It should not contain controller mathematics that belong in `controllers/`.

It should also avoid containing experiment-specific trajectory definitions that belong in `references/` or `experiments/`.

The exact boundary between reusable runner infrastructure and experiment entry points should remain explicit as the repository evolves.

---

## 14. Experiments

Experiments assemble existing framework components into runnable configurations.

An experiment may choose:

```text
Reference Generator
Controller
Controller Configuration
StateProvider
CommandAdapter
Runner Configuration
```

For example:

```text
Circular Reference
        +
VelocityAltitudePIDController
        +
CrazyflieStateProvider
        +
CrazyflieCommandAdapter
        |
        v
Circular Flight Experiment
```

Experiment files should generally be thin.

They should configure and launch reusable components rather than duplicate framework implementation.

---

## 15. Dependency Direction

The intended dependency direction is:

```text
experiments
    |
    v
runner
    |
    +------> controllers
    |
    +------> references
    |
    +------> backends
                |
                v
             hardware
```

Shared components depend on interfaces:

```text
controllers ----\
references ------> interfaces
backends -------/
runner --------/
```

The interfaces package should remain near the center of the architecture and should avoid depending on higher-level components.

New dependencies should generally point inward toward shared interfaces, not outward toward hardware or experiment-specific code.

---

## 16. Architectural Invariants

The following rules should be preserved unless an explicit architectural decision changes them.

### Controller Independence

Controllers must not directly communicate with Crazyflie hardware.

### Hardware Isolation

Crazyflie-specific APIs, units, and command encoding belong in backend components.

### Reference Independence

Reference generators must not depend on a specific controller implementation.

### Experiment Separation

Experiment files configure reusable components but should not become the location of reusable control logic.

### Legacy Isolation

Active framework code must not depend on `legacy/`.

### Shared Units

Framework interfaces should use consistent hardware-independent units.

Current conventions include:

```text
distance        meters
velocity        meters/second
angles          radians
angular rates   radians/second
time            seconds
thrust          normalized [0, 1]
```

Hardware adapters are responsible for converting these values when required.

### Explicit Interface Changes

Changes to public interfaces such as:

```text
DroneState
Reference
AttitudeThrustCommand
Controller
StateProvider
CommandAdapter
```

should be treated as architectural changes.

Related tests and documentation should be updated together.

---

## 17. Current Architecture Status

The current active architecture supports:

* reusable controller interfaces
* velocity and altitude control
* hardware-independent command representation
* Crazyflie-specific state and command backends
* reusable reference generators
* reusable runner infrastructure
* standalone experiments
* automated framework tests

The architecture is still evolving.

Current areas under development include:

* clearer separation between reusable runners and experiment entry points
* yaw and yaw-rate control behavior
* additional state-feedback sources
* motion-capture integration
* additional controller types

Future changes should preserve the separation between control logic, hardware integration, and experiment configuration wherever practical.

---

## 18. Related Documentation

See also:

```text
README.md
docs/control-design.md
AGENTS.md
```

`README.md` provides the project overview.

`control-design.md` documents control equations, coordinate conventions, state assumptions, and individual controller designs.

`AGENTS.md` defines development rules and constraints for coding agents working inside the repository.
