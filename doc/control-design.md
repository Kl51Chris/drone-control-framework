# Control Design

## 1. Purpose

This document describes the control-system design used by the Drone Control Framework.

It focuses on:

* control variables
* state and reference definitions
* coordinate and unit conventions
* controller input and output semantics
* current horizontal velocity control
* current altitude control
* yaw and yaw-rate behavior
* thrust interpretation
* future body-rate control extensions

This document describes control behavior and mathematics. Software ownership and module boundaries are documented separately in `architecture.md`.

---

## 2. Control-System Boundary

The active framework currently treats the controller as a mapping from:

```text
DroneState
+
Reference
+
dt
```

to:

```text
AttitudeThrustCommand
```

Conceptually,

$$
u = f(x, r, \Delta t)
$$

where:

* \(x\) is the estimated vehicle state
* \(r\) is the desired reference
* \(\Delta t\) is the controller update interval
* \(u\) is the controller command

The current command is:

$$
u =
\begin{bmatrix}
\phi_{\mathrm{cmd}} \\
\theta_{\mathrm{cmd}} \\
\dot{\psi}_{\mathrm{cmd}} \\
T_{\mathrm{cmd}}
\end{bmatrix}
$$

where:

* \(\phi_{\mathrm{cmd}}\): roll-angle command
* \(\theta_{\mathrm{cmd}}\): pitch-angle command
* \(\dot{\psi}_{\mathrm{cmd}}\): yaw-rate command
* \(T_{\mathrm{cmd}}\): collective-thrust command

---

## 3. Coordinate and Unit Conventions

The framework uses hardware-independent SI-style units.

### Position

$$
\mathbf{p}
=
\begin{bmatrix}
x \\
y \\
z
\end{bmatrix}
$$

Units:

```text
meters
```

Position is represented in the world frame.

### Velocity

$$
\mathbf{v}
=
\begin{bmatrix}
v_x \\
v_y \\
v_z
\end{bmatrix}
$$

Units:

```text
meters/second
```

Velocity is represented in the world frame.

### Attitude

The framework uses:

$$
\phi = \text{roll}
$$

$$
\theta = \text{pitch}
$$

$$
\psi = \text{yaw}
$$

Units:

```text
radians
```

Interpretation:

```text
roll
    rotation about the body x-axis

pitch
    rotation about the body y-axis

yaw
    rotation about the vertical/body z-axis
```

A useful mnemonic is:

```text
roll  = side-to-side tilt
pitch = nose up/down
yaw   = heading rotation
```

### Angular Rate

Angular rates use:

```text
radians/second
```

For example:

$$
\dot{\psi}
=
\text{yaw rate}
$$

### Thrust

Framework thrust is represented as a normalized scalar:

$$
T_{\mathrm{cmd}} \in [0,1]
$$

This value represents the magnitude of the collective thrust command.

It is not directly a world-frame vertical-force command.

Hardware-specific encoding is performed by the command adapter.

---

## 4. Drone State

The current `DroneState` provides:

```text
position
velocity
roll
pitch
yaw
timestamp
```

Conceptually:

$$
x =
\begin{bmatrix}
\mathbf{p} \\
\mathbf{v} \\
\phi \\
\theta \\
\psi
\end{bmatrix}
$$

The current state does not include measured body angular rates.

This is sufficient for:

* position control
* velocity control
* altitude control
* attitude-angle feedback

but may be insufficient for future controllers that directly require measured:

$$
p,\ q,\ r
$$

or equivalent roll-rate, pitch-rate, and yaw-rate feedback.

---

## 5. Reference

The current `Reference` contains:

```text
position
velocity
yaw
yaw_rate
timestamp
```

Conceptually:

$$
r =
\begin{bmatrix}
\mathbf{p}_{\mathrm{ref}} \\
\mathbf{v}_{\mathrm{ref}} \\
\psi_{\mathrm{ref}} \\
\dot{\psi}_{\mathrm{ref}}
\end{bmatrix}
$$

Not every controller must use every field.

For example, the current horizontal controller uses desired velocity, while the altitude controller uses desired vertical position and velocity.

---

## 6. Current Command Interface

The active command type is:

```text
AttitudeThrustCommand
```

with:

```text
roll
pitch
yaw_rate
thrust
```

Framework-side units are:

```text
roll      radians
pitch     radians
yaw_rate  radians/second
thrust    normalized [0,1]
```

The Crazyflie backend converts these into hardware-specific command units.

The current control boundary therefore sits approximately at the attitude-command level:

```text
framework controller
        |
        v
roll angle
pitch angle
yaw rate
collective thrust
        |
        v
Crazyflie firmware
```

The Crazyflie firmware is responsible for lower-level stabilization.

---

## 7. Horizontal Velocity Control

The current horizontal controller tracks desired world-frame velocity.

Define velocity errors:

$$
e_{v_x}
=
v_{x,\mathrm{ref}} - v_x
$$

$$
e_{v_y}
=
v_{y,\mathrm{ref}} - v_y
$$

The controller converts these errors into desired horizontal accelerations:

$$
a_{x,\mathrm{world}}
=
K_{p,v_x} e_{v_x}
$$

$$
a_{y,\mathrm{world}}
=
K_{p,v_y} e_{v_y}
$$

These accelerations are then transformed using the current yaw angle.

Let:

$$
\psi = \text{current yaw}
$$

Then the desired acceleration is expressed in a yaw-aligned body frame.

A representative rotation is:

$$
a_{x,\mathrm{body}}
=
\cos\psi \, a_{x,\mathrm{world}}
+
\sin\psi \, a_{y,\mathrm{world}}
$$

$$
a_{y,\mathrm{body}}
=
-\sin\psi \, a_{x,\mathrm{world}}
+
\cos\psi \, a_{y,\mathrm{world}}
$$

For small angles, horizontal acceleration is approximately related to attitude by:

$$
a_x \approx g \theta
$$

$$
a_y \approx -g \phi
$$

Therefore:

$$
\theta_{\mathrm{cmd}}
\approx
\frac{a_{x,\mathrm{body}}}{g}
$$

$$
\phi_{\mathrm{cmd}}
\approx
-\frac{a_{y,\mathrm{body}}}{g}
$$

The resulting roll and pitch commands are limited by the configured maximum tilt.

The conceptual control chain is:

```text
vx_ref - vx
vy_ref - vy
        |
        v
desired horizontal acceleration
        |
        v
world-to-body yaw rotation
        |
        v
roll / pitch command
```

---

## 8. Altitude Control

Altitude control uses vertical position and velocity feedback.

Define:

$$
e_z
=
z_{\mathrm{ref}} - z
$$

and:

$$
e_{v_z}
=
v_{z,\mathrm{ref}} - v_z
$$

The vertical controller uses PID-style feedback:

$$
a_{z,\mathrm{fb}}
=
K_{p,z} e_z
+
K_{i,z}
\int e_z \, dt
+
K_{d,z} e_{v_z}
$$

The controller also uses a hover-thrust feedforward term.

Conceptually:

$$
T_{\mathrm{cmd}}
=
T_{\mathrm{hover}}
+
\Delta T_{\mathrm{feedback}}
$$

where:

$$
\Delta T_{\mathrm{feedback}}
=
f(a_{z,\mathrm{fb}})
$$

The hover term represents the approximate collective thrust required to balance gravity.

Without this feedforward term, the controller would need to develop altitude error before generating sufficient thrust to hover.

---

## 9. Interpretation of Thrust

The `thrust` field in `AttitudeThrustCommand` represents total collective thrust magnitude.

It should not be interpreted as a pure world-frame vertical force.

The thrust vector is aligned approximately with the vehicle body thrust axis.

When the vehicle is level:

$$
T_z \approx T
$$

When the vehicle tilts:

$$
T_z
=
T \cos\phi \cos\theta
$$

Therefore part of the available thrust contributes to horizontal acceleration.

This creates coupling between:

```text
roll / pitch
```

and:

```text
altitude
```

even if the software controller treats them as approximately separate channels.

---

## 10. Tilt Compensation

If the controller computes thrust as though the full thrust magnitude acts vertically, altitude can decrease during aggressive roll or pitch motion.

A simple compensation relationship is:

$$
T_{\mathrm{comp}}
=
\frac{T_{\mathrm{vertical}}}
{\cos\phi \cos\theta}
$$

This increases collective thrust as the vehicle tilts so that the vertical component remains approximately constant.

Tilt compensation becomes more important as attitude angles increase.

For small tilt angles, the effect is limited, but it may still contribute to altitude variation during dynamic flight.

The current implementation should explicitly document whether tilt compensation is enabled.

---

## 11. Current Yaw Behavior

The current active controller does not implement a separate yaw-angle feedback loop.

Instead:

$$
\dot{\psi}_{\mathrm{cmd}}
=
\dot{\psi}_{\mathrm{ref}}
$$

Conceptually:

```text
reference.yaw_rate
        |
        v
passthrough
        |
        v
yaw_rate command
```

Therefore `Reference.yaw` may exist without being actively used by the current controller.

This distinction should remain explicit.

---

## 12. Possible Yaw-Angle Control

A future yaw-angle controller may use yaw error:

$$
e_\psi
=
\operatorname{wrap}
(
\psi_{\mathrm{ref}} - \psi
)
$$

and generate a yaw-rate command:

$$
\dot{\psi}_{\mathrm{cmd}}
=
K_{p,\psi} e_\psi
+
\dot{\psi}_{\mathrm{ref}}
$$

This creates the control structure:

```text
yaw reference
      |
      v
yaw-angle error
      |
      v
yaw controller
      |
      v
yaw-rate command
```

This control law requires measured yaw but does not necessarily require measured yaw rate.

---

## 13. Crazyflie Manual Setpoint Modes

The Crazyflie command backend may use a manual setpoint mode that explicitly selects whether roll and pitch are interpreted as angles or angular rates.

Two relevant command modes are:

### Attitude Mode

```text
rate = false
```

The command interpretation is:

```text
roll      angle
pitch     angle
yaw       rate
thrust    collective thrust
```

Conceptually:

$$
\phi_{\mathrm{cmd}},
\theta_{\mathrm{cmd}},
\dot{\psi}_{\mathrm{cmd}},
T_{\mathrm{cmd}}
$$

This matches the current `AttitudeThrustCommand` abstraction.

### Body-Rate Mode

```text
rate = true
```

The command interpretation becomes:

```text
roll_rate
pitch_rate
yaw_rate
thrust
```

Conceptually:

$$
\dot{\phi}_{\mathrm{cmd}},
\dot{\theta}_{\mathrm{cmd}},
\dot{\psi}_{\mathrm{cmd}},
T_{\mathrm{cmd}}
$$

This represents a lower-level control boundary.

---

## 14. Attitude Command vs Body-Rate Command

The two control boundaries should be treated as distinct command types.

### Attitude-Level Control

```text
outer controller
      |
      v
desired roll / pitch
      |
      v
AttitudeThrustCommand
      |
      v
Crazyflie attitude control
```

The framework delegates roll and pitch attitude stabilization to the Crazyflie firmware.

### Body-Rate-Level Control

```text
outer controller
      |
      v
desired attitude
      |
      v
framework attitude controller
      |
      v
desired body rates
      |
      v
BodyRateThrustCommand
      |
      v
Crazyflie body-rate control
```

In this architecture, the framework assumes responsibility for the attitude-to-rate control layer.

---

## 15. Proposed BodyRateThrustCommand

If body-rate control is added, it should use a separate command representation.

Conceptually:

```text
BodyRateThrustCommand
```

with:

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
thrust      normalized [0,1]
```

This command should remain distinct from `AttitudeThrustCommand`.

The distinction is important because:

```text
roll = 0.2
```

and:

```text
roll_rate = 0.2
```

represent fundamentally different physical commands.

---

## 16. Cascaded Control Interpretation

The control system can be understood as a set of nested loops.

A high-level hierarchy is:

```text
position
   |
   v
velocity
   |
   v
acceleration
   |
   v
attitude
   |
   v
body rate
   |
   v
torque
   |
   v
motor thrust
```

The current framework operates primarily above the attitude-command boundary.

Using body-rate commands moves the framework boundary one level lower.

This does not directly command angular acceleration or torque.

The Crazyflie firmware still performs body-rate stabilization and motor mixing.

---

## 17. Motor Mixing

The four Crazyflie motors ultimately receive different motor commands.

At a simplified level, the lower-level controller determines:

$$
T
$$

and rotational moments:

$$
\tau_x,\ \tau_y,\ \tau_z
$$

These correspond approximately to:

```text
collective thrust
roll torque
pitch torque
yaw torque
```

A motor mixer then converts these into:

$$
m_1,\ m_2,\ m_3,\ m_4
$$

The framework's collective thrust command therefore acts together with attitude or rate control outputs.

The individual motor commands are not currently exposed as framework-level control outputs.

---

## 18. Controller Time-Scale Separation

For cascaded controllers, inner loops should generally operate faster than outer loops.

For example:

```text
velocity loop
    slower

attitude loop
    faster

body-rate loop
    faster still
```

If the framework begins implementing its own attitude controller while the Crazyflie firmware retains the body-rate loop, the attitude controller bandwidth should be chosen with this hierarchy in mind.

Poor time-scale separation can cause oscillation or conflicting control action.

---

## 19. Current Limitations

The current control design has several known limitations.

### No Explicit Body-Rate State

`DroneState` currently does not expose measured angular rates.

This limits direct implementation of controllers that require:

$$
p,\ q,\ r
$$

feedback.

### Yaw Is Mostly Open at the Framework Level

The current controller passes yaw-rate reference directly to the command output.

Yaw-angle tracking is not yet implemented.

### Horizontal and Vertical Dynamics Are Only Approximately Decoupled

Roll and pitch affect the vertical thrust component.

Altitude performance may therefore change during aggressive horizontal motion.

### Tilt Compensation Requires Explicit Treatment

The current control design should clearly state whether collective thrust is corrected for vehicle tilt.

### Hardware Inner Loops Remain Active

Even when the framework generates body-rate commands, Crazyflie firmware still performs lower-level stabilization.

The controlled plant is therefore not a completely raw rigid-body system.

---

## 20. Design Principles for Future Controllers

Future controllers should preserve the following principles.

### Explicit Physical Meaning

Every command field must have a clearly defined physical meaning and unit.

### Separate Command Levels

Different control levels should use distinct command types where their physical semantics differ.

For example:

```text
AttitudeThrustCommand
```

and:

```text
BodyRateThrustCommand
```

should not be silently treated as interchangeable.

### Hardware-Independent Units

Controller-side commands should remain in consistent framework units.

Hardware conversion should occur in the backend.

### State Requirements Must Be Explicit

A controller requiring angular-rate feedback, acceleration, or motion-capture position must declare that requirement clearly.

### Controller Boundaries Should Be Intentional

Moving from attitude commands to body-rate commands changes the control-system boundary and should be treated as a deliberate design decision rather than a simple API substitution.

---

## 21. Current Control Summary

The current controller can be summarized as:

```text
Horizontal:

velocity reference
      |
      v
velocity error
      |
      v
desired horizontal acceleration
      |
      v
roll / pitch angle command
```

```text
Vertical:

altitude + vertical velocity reference
      |
      v
PID feedback
      |
      +
hover thrust feedforward
      |
      v
collective thrust command
```

```text
Yaw:

yaw-rate reference
      |
      v
passthrough
      |
      v
yaw-rate command
```

The complete current command is:

$$
u =
\begin{bmatrix}
\phi_{\mathrm{cmd}} \\
\theta_{\mathrm{cmd}} \\
\dot{\psi}_{\mathrm{cmd}} \\
T_{\mathrm{cmd}}
\end{bmatrix}
$$

Future body-rate control would instead use:

$$
u =
\begin{bmatrix}
\dot{\phi}_{\mathrm{cmd}} \\
\dot{\theta}_{\mathrm{cmd}} \\
\dot{\psi}_{\mathrm{cmd}} \\
T_{\mathrm{cmd}}
\end{bmatrix}
$$

These two command levels should remain explicitly distinguished.
