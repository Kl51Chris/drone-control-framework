# Drone Control Framework Agent Instructions

## 1. Project Purpose

This repository contains a modular drone control framework for Crazyflie-based control experiments.

The primary design goal is to separate:

* control algorithms
* trajectory/reference generation
* vehicle-state acquisition
* hardware-specific command conversion
* reusable execution infrastructure
* experiment-specific configuration

The framework should support multiple controller implementations without requiring unrelated parts of the system to be rewritten.

---

## 2. Source of Truth

Before making architectural changes, inspect:

* `README.md`
* `docs/architecture.md`
* `docs/control-design.md`
* the current implementation under `velocity_control_framework/`

Code and documentation must remain consistent.

Do not assume that historical or legacy code represents the current architecture.

---

## 3. Repository Responsibilities

### `velocity_control_framework/`

Contains reusable framework components.

Reusable controller, reference, backend, interface, runner, and test logic belongs here.

### `experiments/`

Contains runnable experiments and experiment-specific configuration.

Experiment files should assemble reusable components rather than implement reusable control logic directly.

### `tools/`

Contains diagnostics, hardware checks, inspection utilities, and simulation utilities.

Tools are not part of the normal controller execution path.

### `legacy/`

Contains historical implementations retained for reference.

Do not modify files under `legacy/` unless explicitly requested.

New framework code must not depend on modules under `legacy/`.

### `docs/`

Contains detailed technical documentation.

Update relevant documentation when public interfaces, architecture, units, or control behavior change.

---

## 4. Core Architecture

The intended control flow is:

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
             Command
                  |
                  v
            CommandAdapter
                  |
                  v
              Hardware
```

State feedback follows:

```text
Hardware
   |
   v
StateProvider
   |
   v
DroneState
```

The runner coordinates these components.

---

## 5. Controller Contract

Controllers are reusable control components.

A controller should conceptually perform:

```text
DroneState
+
Reference
+
dt
    |
    v
Control Command
```

Controllers may:

* compute feedback errors
* maintain internal controller state
* integrate errors
* solve optimization problems
* apply controller-specific limits

Controllers must not:

* directly call Crazyflie APIs
* directly access Crazyradio
* open or close hardware connections
* directly read raw Crazyflie log variables
* depend on experiment entry points
* depend on `legacy/`

Hardware-specific behavior belongs in backend components.

---

## 6. State Provider Contract

State providers convert hardware or estimator data into the common `DroneState` representation.

Controllers should consume `DroneState`, not raw sensor or firmware variables.

The current Crazyflie state provider obtains estimator values from Crazyflie logging and converts them into framework units.

Future state sources, such as motion capture or simulation, should preserve the same abstraction wherever practical.

---

## 7. Reference Contract

Reference generators describe desired vehicle motion.

Reference generation should remain independent of the controller implementation.

A reference generator should not contain PID-, LQR-, MPC-, or hardware-specific logic.

Controllers determine how to track a reference.

Reference generators determine what motion is desired.

---

## 8. Hardware Boundary

Hardware-specific behavior belongs under backend implementations.

Examples include:

* Crazyflie API calls
* command packet selection
* hardware-specific units
* unit conversions
* sign conventions
* thrust encoding
* firmware-specific behavior
* Crazyflie logging configuration

These details must not leak into reusable controllers.

---

## 9. Framework Units

Unless explicitly documented otherwise, framework-facing interfaces should use:

```text
distance        meters
velocity        meters/second
angles          radians
angular rates   radians/second
time            seconds
thrust          normalized framework representation
```

Hardware-specific adapters are responsible for required conversion.

Never silently change the physical meaning or units of an existing public field.

If such a change is necessary, update:

* implementation
* tests
* relevant documentation

in the same change.

---

## 10. Public Interfaces

Treat changes to the following as architectural changes:

* `DroneState`
* `Reference`
* controller protocols
* command data structures
* `StateProvider`
* `CommandAdapter`

Before modifying these interfaces:

1. inspect all current users
2. preserve compatibility where practical
3. update affected tests
4. update `docs/architecture.md`
5. update `docs/control-design.md` when the physical/control meaning changes

Do not introduce a new public abstraction unless it represents a genuinely different responsibility or physical command level.

---

## 11. Experiments

Experiments should remain thin.

They may:

* instantiate controllers
* configure controller parameters
* choose references
* configure runners
* select hardware backends
* define experiment duration and flight sequence

They should not duplicate reusable framework logic.

If logic could reasonably be reused by another experiment, place it in the framework instead.

---

## 12. Runner Responsibilities

Runner code is orchestration code.

Runners may:

* obtain state
* obtain reference
* compute update timing
* call controllers
* send commands
* manage experiment execution
* perform safe shutdown behavior

Runners should not contain reusable controller mathematics.

Avoid embedding trajectory-generation logic directly in runners when it belongs in a reference generator.

---

## 13. Testing

After modifying reusable framework code, run:

```bash
pytest velocity_control_framework/tests -v
```

Do not report a framework change as complete if relevant tests fail.

When adding or changing a public interface, add or update tests that verify the new contract.

Hardware tests under `tools/` are not substitutes for framework unit tests.

---

## 14. Real Hardware Safety

Do not automatically execute real flight tests.

Do not automatically:

* arm motors
* take off
* send sustained motor commands
* modify flight-control gains
* increase attitude/rate limits
* increase thrust limits
* change emergency-stop behavior

unless explicitly requested.

When modifying hardware-facing commands, preserve existing safety limits unless the task specifically requires changing them.

Software tests passing does not imply that a controller is safe for real flight.

---

## 15. Control-Design Changes

Do not confuse:

* API changes
* command encoding changes
* physical control-level changes

A backend API substitution does not automatically imply a new controller architecture.

A change from attitude commands to body-rate commands, torque commands, or individual motor commands represents a different physical command level and should be treated explicitly.

Do not silently reinterpret existing command fields.

---

## 16. Documentation Maintenance

When code changes affect any of the following:

* repository structure
* module responsibility
* control flow
* public interfaces
* coordinate conventions
* units
* control equations
* state availability
* command semantics

update the relevant Markdown documentation in the same branch.

Use:

```text
README.md
```

for project-level overview.

Use:

```text
docs/architecture.md
```

for software responsibilities, dependencies, and architectural boundaries.

Use:

```text
docs/control-design.md
```

for control equations, state/reference/command semantics, units, and control-loop behavior.

Do not duplicate large sections of documentation across multiple files.

---

## 17. Legacy Code

Legacy code exists for historical reference only.

Do not:

* import legacy modules into active framework code
* move new implementation into `legacy/`
* modify legacy behavior as part of unrelated work

If useful historical logic is needed, reimplement it cleanly within the active architecture rather than introducing a dependency on legacy code.

---

## 18. Change Discipline

Prefer small, reviewable changes.

Before changing multiple architectural layers, determine which layers actually need modification.

For example, a hardware API change may only require a backend change and tests.

Do not refactor unrelated modules merely because they are nearby.

Avoid speculative abstractions.

Introduce generalization when there is a concrete requirement for it.

---

## 19. Completion Criteria

Before considering a code change complete:

* inspect the resulting diff
* ensure architectural boundaries are preserved
* run relevant tests
* remove temporary debugging code
* update affected documentation
* summarize what changed
* identify any remaining hardware-validation requirements

Do not claim real-flight validation unless a real hardware test was explicitly performed and confirmed.
