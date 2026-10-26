# Control Design

## 1. Control System Overview

### 1.1 Control Objectives and Scope

The framework supports interchangeable offboard controllers for Crazyflie
position and trajectory tracking. At the shared control boundary, an offboard
controller maps the current vehicle state and desired reference to commanded
body angular rates and normalized collective thrust. Crazyflie firmware closes
the onboard body-rate loops and performs motor mixing; the offboard command is
not a direct motor command.

`OuterLoopPIDController` is the currently implemented reusable controller. LQR,
MPC, and DeePC are planned future controllers and are not implemented. See
[`architecture.md`](architecture.md) for software architecture, component
boundaries, state acquisition, hardware adaptation, and execution flow.

### 1.2 State, Reference, and Command Definitions

The common `DroneState` representation provides

```math
\mathbf{s} =
\begin{bmatrix}
p_x & p_y & p_z &
v_x & v_y & v_z &
\phi & \theta & \psi &
p & q & r
\end{bmatrix}^{\mathsf T}.
```

Position `(p_x, p_y, p_z)` and velocity `(v_x, v_y, v_z)` are world-frame
quantities. Attitude is represented by roll, pitch, and yaw Euler angles
`(phi, theta, psi)`. The values `(p, q, r)` are measured or estimated angular
rates about the body x, y, and z axes. Body rates are not generally equal to
the Euler-angle derivatives `(phi_dot, theta_dot, psi_dot)`.

`DroneState` is a common software representation, not a mandated mathematical
state vector for every controller. A controller may select or derive the state
variables appropriate to its formulation. The representation also includes a
timestamp in seconds, which is omitted from the vector above because it is
sample metadata rather than a dynamic state.

The common `Reference` is

```math
\mathbf{r}_{ref} =
\begin{bmatrix}
\mathbf{p}_{ref} &
\mathbf{v}_{ref} &
\psi_{ref} &
\dot{\psi}_{ref}
\end{bmatrix}^{\mathsf T},
```

where `position = p_ref` and `velocity = v_ref` are desired world-frame
three-vectors, `yaw = psi_ref` is desired world-frame yaw, and
`yaw_rate = psi_ref_dot` is desired yaw rate. `Reference` also carries a
timestamp in seconds. The cascaded PID uses position, velocity, and yaw;
vertical reference velocity is used when its configuration enables reference
velocity tracking. The current production candidate uses zero-reference
vertical-velocity damping and does not add yaw-rate feed-forward.

The shared controller output is

```math
\mathbf{u} =
\begin{bmatrix}
p_{cmd} & q_{cmd} & r_{cmd} & T_{cmd}
\end{bmatrix}^{\mathsf T}.
```

The first three elements are desired body angular rates; `T_cmd` is normalized
collective thrust. Crazyflie firmware interprets the rate references through
its lower-level stabilization and motor-mixing logic.

| Quantity | Convention | Unit |
|---|---|---|
| Position `p`, reference `p_ref` | World frame | m |
| Velocity `v`, reference `v_ref` | World frame | m/s |
| Euler attitude `(phi, theta, psi)` | Roll, pitch, world-frame yaw | rad |
| State body rates `(p, q, r)` | About body x, y, z | rad/s |
| Command rates `(p_cmd, q_cmd, r_cmd)` | About body x, y, z | rad/s |
| Normalized thrust `T_cmd` | Collective command in `[0, 1]` | dimensionless |
| Timestamps and controller `dt` | Time | s |

## 2. Outer-Loop PID Controller [Implemented]

### 2.1 Controller Overview and Block Diagram

`OuterLoopPIDController` implements the cascade migrated from the calibration
toolkit. World-frame XY position/velocity feedback produces a bounded desired
horizontal acceleration. `R(-psi)` rotates that vector into body coordinates;
experimentally identified signs map body-X acceleration to pitch reference and
body-Y acceleration to roll reference. Roll/pitch attitude-P feedback produces
body-rate commands. Altitude PD acts around equilibrium thrust, and wrapped
heading feedback produces yaw rate.

The firmware body-rate PID and mixer remain downstream. The controller owns no
takeoff, landing, arming, or flight-state logic. `HorizontalCommandMapping`
retains the former direct position-to-rate behavior solely for existing staged
validation runners; new cascaded configurations use
`HorizontalAccelerationMapping`.

### 2.2 Position Error and Coordinate Transformation

World-frame horizontal position and velocity errors are

```math
e_x^W = p_{x,ref} - p_x,
\qquad
e_y^W = p_{y,ref} - p_y,
\qquad
e_{v,x}^W=v_{x,ref}-v_x,
\qquad
e_{v,y}^W=v_{y,ref}-v_y.
```

The horizontal channels produce world-frame acceleration requests

```math
a_x^W=K_{p,x}e_x^W+K_{i,x}I_x+K_{d,x}e_{v,x}^W,
\qquad
a_y^W=K_{p,y}e_y^W+K_{i,y}I_y+K_{d,y}e_{v,y}^W.
```

After vector-magnitude limiting, acceleration is rotated into the horizontal
body frame with `R(-psi)`:

```math
\begin{bmatrix}
a_x^B \\
a_y^B
\end{bmatrix}
=
\begin{bmatrix}
\cos\psi & \sin\psi \\
-\sin\psi & \cos\psi
\end{bmatrix}
\begin{bmatrix}
a_x^W \\
a_y^W
\end{bmatrix}.
```

Equivalently,

```math
a_x^B = \cos\psi\,a_x^W + \sin\psi\,a_y^W,
\qquad
a_y^B = -\sin\psi\,a_x^W + \cos\psi\,a_y^W.
```

Only yaw enters this planar transformation.

### 2.3 PID/PD Channel Behavior

For channel

```math
i \in \{x_W, y_W, z, \psi\},
```

the unsaturated PID output is

```math
u_{i,k}^{*}
= K_{p,i} e_{i,k}
+ K_{i,i} I_{i,k}^{*}
+ K_{d,i} d_{i,k}.
```

For cascaded XY and altitude, `d_i` is the corresponding measured velocity
error. Heading retains successive wrapped-error differentiation if a nonzero
heading `kd` is configured.

For a nonzero integral gain, the candidate integral uses rectangular
integration:

```math
I_{i,k}^{*}=I_{i,k-1}+e_{i,k}\Delta t.
```

When `K_i = 0`, the stored integral is left unchanged. When an explicit
velocity error is not supplied, the derivative is taken from successive
errors:

```math
d_{i,k}=
\begin{cases}
0, & \text{first update or }K_{d,i}=0,\\
\dfrac{e_{i,k}-e_{i,k-1}}{\Delta t}, & \text{otherwise}.
\end{cases}
```

The implementation does not apply derivative filtering or setpoint weighting.

### 2.4 Acceleration, Attitude, and Body-Rate Mapping

Body acceleration maps to bounded attitude references as

```math
\theta_{ref}=s_x\operatorname{atan2}(a_x^B,g),
\qquad
\phi_{ref}=s_y\operatorname{atan2}(a_y^B,g).
```

Hardware identification supports `s_x=-1` and `s_y=-1`. The attitude loop is

```math
p_{cmd}=s_{\phi}K_{\phi}(\phi_{ref}-\phi),
\qquad
q_{cmd}=s_{\theta}K_{\theta}(\theta_{ref}-\theta),
```

with independent rate limits. The Stage 5 command convention uses
`s_phi=+1` and `s_theta=-1`; the latter accounts for the existing cflib command
path. These command signs are deliberately separate from the experimentally
measured physical acceleration-to-attitude signs.

### 2.5 Altitude and Thrust Control

The altitude channel operates on

```math
e_z=p_{z,ref}-p_z,
\qquad
e_{v,z}=v_{z,desired}-v_z,
\qquad
T_{feedback}=K_{p,z}e_z+K_{i,z}I_z+K_{d,z}e_{v,z}.
```

By default `v_z,desired=0` implements zero-reference velocity damping. A
configuration flag can instead track `Reference.velocity[2]`.

It combines the PID correction with equilibrium-thrust feed-forward:

```math
T_{cmd}=\operatorname{clamp}
\left(
T_{equilibrium}+T_{feedback},
T_{min},
T_{max}
\right).
```

`T_equilibrium` is the normalized collective command intended to balance the
nominal vehicle at the operating condition; the altitude channel adjusts around
it. Configuration requires
`0 <= T_min <= T_equilibrium <= T_max <= 1`. The altitude correction limits
are intersected with available upward/downward thrust authority before the PID
is constructed. Conditional integration therefore cannot wind up against a
tighter final thrust bound.

Normalized thrust is dimensionless command magnitude, not force in newtons and
not a world-frame vertical-force command.

### 2.6 Heading Control

The yaw error is wrapped to the principal interval using

```math
e_{\psi}=\operatorname{atan2}
\left(
\sin(\psi_{ref}-\psi),
\cos(\psi_{ref}-\psi)
\right).
```

The heading channel produces

```math
r_{cmd}=PID_{\psi}(e_{\psi}),
```

which is a commanded angular rate about body z. The input error is formed from
Euler yaw angles, but the output is a body-axis rate command; it is not a
commanded Euler yaw derivative. The controller does not add
`Reference.yaw_rate` feed-forward and does not feed back measured body rate
`DroneState.r`.

### 2.7 Saturation and Anti-Windup

Each PID channel applies its configured output limits:

```math
u_{i,k}=\operatorname{clamp}
\left(u_{i,k}^{*},u_{i,min},u_{i,max}\right).
```

The implementation uses conditional integration. The candidate integral
`I_{i,k}^{*}` is committed when the candidate output is unsaturated. When saturated,
it is committed only if the current integral drive `K_i e_i,k` would move an
above-maximum output downward or a below-minimum output upward. Otherwise the
stored integral remains unchanged:

```math
I_{i,k}=
\begin{cases}
I_{i,k}^{*}, & u_{i,k}^{*}=u_{i,k},\\
I_{i,k}^{*}, & u_{i,k}^{*}>u_{i,max}\ \text{and}\ K_{i,i}e_{i,k}<0,\\
I_{i,k}^{*}, & u_{i,k}^{*}<u_{i,min}\ \text{and}\ K_{i,i}e_{i,k}>0,\\
I_{i,k-1}, & \text{otherwise}.
\end{cases}
```

This update occurs only when `K_i != 0`. Previous error is stored on every
update, including saturated updates. There is no separate integral-state clamp
or back-calculation term.

In cascaded mode, horizontal acceleration is limited by vector magnitude after
the two world-frame components are combined; it is not clipped per axis first.
This preserves the requested acceleration direction. Roll/pitch references are
vector-limited, and attitude body-rate commands have independent limits.
Heading inherits its PID limits. Altitude uses the effective correction limits
described in Section 2.5. The hardware-tested candidate has zero integral gain
in XY and altitude.

### 2.8 Implementation and Validation Status

The cascaded controller, measured-velocity PD feedback, world-to-body
transformation, identified physical mappings, attitude P loop, saturation,
conditional-integration anti-windup, equilibrium thrust, and heading wrapping
are implemented. Automated tests cover both restoring directions, yaw rotation,
attitude command conventions, altitude damping, limits, and legacy-runner
compatibility.

Calibration hardware runs support the two physical mapping signs and bounded
XY/Z hover behavior. They do not establish final tuning or reliable touchdown;
the Stage 5 candidate factory is therefore an explicit experimental example.

Stage 2 of the staged experiment uses a separate, experiment-specific roll,
pitch, and yaw attitude P controller with rate limiting and an equilibrium-
thrust ramp. It is not part of `OuterLoopPIDController`. Its authoritative
implementation is
[`experiments/outer_loop_pid_validation.py`](../experiments/outer_loop_pid_validation.py),
principally `OuterLoopPIDValidationProgram._stage_two_command()` and
`_stage_two_thrust()`.

The frozen Stage 6 runner uses the legacy `HorizontalCommandMapping` mode and
adds an experiment-specific roll/pitch attitude-feedback wrapper:

```math
p_{cmd}=\operatorname{sat}_{L_{att}}
\left(p_{PID}+s_{\phi}K_{\phi}(0-\phi)\right),
\qquad
q_{cmd}=\operatorname{sat}_{L_{att}}
\left(q_{PID}+s_{\theta}K_{\theta}(0-\theta)\right).
```

The wrapper preserves the controller's yaw-rate and thrust outputs. Its final
roll/pitch saturation is applied after summation; it does not remove or revise
the reusable PID channels' earlier saturation or anti-windup decisions. Stage 6
uses smooth altitude references for takeoff and landing, and normal completion
requires measured altitude and vertical velocity to satisfy configured landing
tolerances. This is a landing-state proxy, not independent proof of physical
touchdown; the available state interface has no dedicated contact signal.
These mechanisms remain experiment-specific and are not evidence of
hardware-validated free-flight safety.

## 2.9 Software responsibilities and extensions

`experiments/run_flight.py` is the small assembly point and CLI: it selects the
production controller configuration and reference provider.
`ProductionFlightProgram` owns the fixed-hover phase profile and lifecycle
checks. `UniversalRunner` owns state-provider startup, monotonic timing,
control-loop scheduling, command transmission, logging callbacks, stop, and
cleanup. The controller only computes a command from `DroneState`, `Reference`,
and `dt`; it does not arm, land, or call cflib. A reference provider only
generates desired values. The state provider acquires the latest state, and the
command adapter owns conversion to Crazyflie packet units.

To prototype MPC or DeePC, implement the existing controller interface and
return `BodyRateThrustCommand`; keep horizons, gains, constraints, and other
parameters in a configuration object; add explicit selection in the experiment
entry point; then add unit, interface, and offline-history tests before any
hardware test. Do not implement a second control loop. To add a trajectory,
implement the existing `Reference`-producing provider without changing the
controller or `UniversalRunner`; `references/circular_flight_reference.py` is
the current example.

## 3. LQR Controller [Planned]

Linear-quadratic regulation is intended to use a linearized model and quadratic
state/input objectives while preserving the common controller input/output
boundary. The operating point, internal
state selection, model derivation, cost design, constraint handling, and
reference-tracking formulation remain unresolved. The internal mathematical
state need not equal the complete `DroneState` representation. **Status: Not
implemented.**

## 4. MPC Controller [Planned]

Model predictive control is intended to optimize predicted tracking behavior
and control effort over a finite horizon while returning a
`BodyRateThrustCommand` through the common interface. The prediction model,
internal state, horizon, objective, constraints, estimator coupling, and
real-time solution strategy remain unresolved. **Status: Not implemented.**

## 5. DeePC Controller [Planned]

Data-enabled predictive control is intended to compute constrained predictive
commands from recorded input/output behavior while using the same external
controller interface. Required excitation data, signal selection, data quality
criteria, regularization, constraints, horizon, and online optimization method
remain unresolved. Its internal variables need not match those of PID, LQR, or
MPC. **Status: Not implemented.**
