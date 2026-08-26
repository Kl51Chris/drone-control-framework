# Body-Rate Control Proposal

## Status

**Proposal only. Not currently implemented.**

This document describes a possible future extension of the Drone Control Framework.

Nothing in this document should be treated as current system behavior unless the corresponding implementation has been merged into the active codebase and the current control-design documentation has been updated.

---

## 1. Motivation

The current framework outputs an attitude-level command containing:

```text
roll
pitch
yaw_rate
thrust
```

A possible future extension is to move the framework control boundary one level lower and directly command body angular rates.

The proposed command would contain:

```text
roll_rate
pitch_rate
yaw_rate
thrust
```

This would allow the framework to implement its own attitude-to-rate control layer while continuing to rely on the Crazyflie firmware for lower-level body-rate stabilization and motor mixing.

---

## 2. Proposed Control Boundary

Current concept:

```text
Outer Controller
      |
      v
roll / pitch attitude command
      |
      v
Crazyflie attitude controller
      |
      v
Crazyflie rate controller
```

Proposed concept:

```text
Outer Controller
      |
      v
desired attitude
      |
      v
framework attitude controller
      |
      v
body-rate command
      |
      v
Crazyflie rate controller
```

The proposed architecture therefore moves the framework boundary from attitude command to body-rate command.

---

## 3. Proposed Command Type

A new command type may be introduced:

```text
BodyRateThrustCommand
```

with fields:

```text
roll_rate
pitch_rate
yaw_rate
thrust
```

Recommended framework units:

```text
roll_rate   rad/s
pitch_rate  rad/s
yaw_rate    rad/s
thrust      normalized framework representation
```

This command should remain distinct from the existing attitude-level command because angle and angular-rate commands have different physical meanings.

---

## 4. Proposed Controller Structure

A possible cascaded structure is:

```text
position / velocity reference
        |
        v
outer-loop controller
        |
        v
desired attitude
        |
        v
attitude controller
        |
        v
desired body rate
        |
        v
Crazyflie body-rate controller
```

For example, an attitude controller could generate:

$$
\dot{\phi}_{cmd}
=
K_{p,\phi}
(\phi_{ref}-\phi)
$$

$$
\dot{\theta}_{cmd}
=
K_{p,\theta}
(\theta_{ref}-\theta)
$$

and possibly:

$$
\dot{\psi}_{cmd}
=
K_{p,\psi}
\operatorname{wrap}
(\psi_{ref}-\psi)
+
\dot{\psi}_{ref}
$$

These equations are illustrative only and are not part of the current implementation.

---

## 5. State Requirements

The current `DroneState` includes attitude angles but does not expose measured body angular rates.

A basic attitude-to-rate controller can be implemented using attitude feedback alone.

However, more advanced rate-feedback controllers may require measured:

```text
roll_rate
pitch_rate
yaw_rate
```

or body rates:

```text
p
q
r
```

If such controllers are implemented, the state interface may need to be extended.

That change should be treated as a public-interface modification and reviewed separately.

---

## 6. Crazyflie Command Backend

A possible implementation may use a Crazyflie command API that explicitly selects rate control for roll and pitch.

In such a mode, the backend would convert framework angular-rate units into Crazyflie-specific units before transmission.

Any API selection, unit conversion, thrust encoding, and firmware-specific behavior must remain inside the backend layer.

The controller should remain hardware-independent.

---

## 7. Expected Benefits

Potential benefits include:

* direct control over attitude-loop behavior
* easier experimentation with custom attitude controllers
* clearer access to lower-level vehicle dynamics
* improved suitability for certain LQR or MPC designs
* explicit separation between attitude-level and rate-level control experiments

These are expected benefits only and require experimental validation.

---

## 8. Expected Drawbacks

Potential disadvantages include:

* additional controller tuning
* greater sensitivity to command timing and update rate
* reduced reliance on the Crazyflie firmware's built-in attitude controller
* increased risk during real-flight testing
* possible need for additional state feedback
* more complex command and adapter abstractions

Moving the control boundary lower does not automatically improve flight performance.

---

## 9. Open Questions

The following issues must be resolved before implementation:

* whether body-rate control is required for the intended experiments
* whether the current state feedback is sufficient
* required control-loop frequency
* command saturation limits
* attitude-controller structure
* interaction with the existing velocity controller
* thrust encoding and compatibility with existing hover calibration
* whether the controller interface should become generic over command type
* whether separate command adapters should be used for attitude and rate commands

---

## 10. Implementation Rule

This proposal must not modify the documented meaning of the current controller until implementation is complete.

If the proposal is implemented:

1. create the necessary command and controller abstractions
2. update automated tests
3. validate software behavior
4. update `docs/control-design.md`
5. update `docs/architecture.md` if architectural boundaries change
6. mark this proposal as implemented, superseded, or archived

Until then, this document remains informational only.
