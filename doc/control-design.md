# Control Design

## 1. Active Control Boundary

```text
DroneState + Reference + dt
           |
           v
  OuterLoopPIDController
           |
           v
 BodyRateThrustCommand
```

`dt` is elapsed update time in seconds and must be finite and positive.
`reset()` clears every PID channel's integral and previous error.

`DroneState.roll`, `.pitch`, and `.yaw` are Euler attitude angles in radians.
`DroneState.p`, `.q`, and `.r` are angular rates about the body x, y, and z axes
in radians per second. Except in special attitudes, body rates are not equal to
Euler-angle derivatives. Command rates are desired body-axis rates; they are
neither attitude-angle nor Euler-angle-derivative commands.

The controller uses `Reference.position` and `Reference.yaw`. It does not use
the available desired velocity or yaw-rate feed-forward fields.

## 2. Position Error and Horizontal Coordinates

Let the world/inertial horizontal position error be

```math
e_x^I = x_{ref} - x, \qquad e_y^I = y_{ref} - y.
```

Using measured yaw `psi`, the implementation rotates it by `R(-psi)`:

```math
\begin{bmatrix} e_x^B \\ e_y^B \end{bmatrix}
=
\begin{bmatrix}
\cos\psi & \sin\psi \\
-\sin\psi & \cos\psi
\end{bmatrix}
\begin{bmatrix} e_x^I \\ e_y^I \end{bmatrix}.
```

Equivalently,

```math
e_x^B = \cos\psi\,e_x^I + \sin\psi\,e_y^I,
```

```math
e_y^B = -\sin\psi\,e_x^I + \cos\psi\,e_y^I.
```

Only yaw enters this planar transform; roll and pitch do not.

## 3. Implemented PID Numerics

Four independent PID instances process body-horizontal X, body-horizontal Y,
altitude, and heading error. Each has gains `(K_p, K_i, K_d)` and configured
output bounds `[u_min, u_max]`.

For update `k`, the derivative is computed from error:

```math
d_k =
\begin{cases}
0, & k=0 \text{ or } K_d=0,\\
(e_k-e_{k-1})/\Delta t, & \text{otherwise}.
\end{cases}
```

When `K_i != 0`, a rectangular-rule candidate integral is

```math
I_k^* = I_{k-1} + e_k\Delta t.
```

When `K_i = 0`, the stored integral is unchanged. The candidate and returned
outputs are

```math
u_k^* = K_p e_k + K_i I_k^* + K_d d_k,
\qquad
u_k = \operatorname{clamp}(u_k^*, u_{min}, u_{max}).
```

Conditional-integration anti-windup commits `I_k = I_k^*` if the candidate is
unsaturated, or if integral drive `K_i e_k` would move an over-limit output
downward or an under-limit output upward. Otherwise it keeps the old integral.
Previous error is updated on every call, including saturated calls. There is no
separate integral clamp, derivative filter, setpoint weighting, or
back-calculation term.

## 4. Horizontal Channels and Body-Rate Mapping

```math
u_x = PID_x(e_x^B), \qquad u_y = PID_y(e_y^B).
```

The implemented mapping is

```math
q_{cmd} = s_x u_x, \qquad p_{cmd} = s_y u_y,
```

where `q_cmd` is `pitch_rate`, `p_cmd` is `roll_rate`,
`s_x = body_x_to_pitch_rate_sign`, and
`s_y = body_y_to_roll_rate_sign`. Thus body X maps to pitch rate and body Y to
roll rate. Each sign must be `-1` or `+1`.

These polarities are explicit configuration, not established universal hardware
signs. Tests verify application of configured signs; they do not physically
verify the correct signs for a vehicle.

## 5. Altitude and Collective Thrust

```math
e_z = z_{ref} - z, \qquad u_z = PID_z(e_z).
```

The equilibrium-thrust feed-forward and feedback are combined as

```math
T_{command} = \operatorname{clamp}
\left(T_{equilibrium} + u_z, T_{min}, T_{max}\right).
```

All thrust values are normalized to `[0, 1]`, with
`T_min <= T_equilibrium <= T_max`. The altitude PID limits `u_z` before the
overall thrust clamp. Normalized thrust is a command magnitude, not force in
newtons or a world-frame vertical force. `BodyRateThrustCommand.zero()` means
zero motor thrust, not hover thrust.

## 6. Heading Channel

Heading error is wrapped using the exact implementation:

```math
e_\psi = \operatorname{atan2}
\left(\sin(\psi_{ref}-\psi), \cos(\psi_{ref}-\psi)\right).
```

This selects the principal error in `[-pi, pi]`, subject to the usual `atan2`
endpoint representation. The output is

```math
r_{cmd} = PID_\psi(e_\psi).
```

The controller neither adds `Reference.yaw_rate` nor feeds back
`DroneState.r`.

## 7. Crazyflie Backend and Onboard Responsibilities

The adapter converts each framework angular rate independently:

```math
\omega_{deg/s} = \omega_{rad/s}\frac{180}{\pi}.
```

It clamps normalized thrust to `[0, 1]`, multiplies by 100, and calls:

```text
send_setpoint_manual(
    roll_rate_deg_s,
    pitch_rate_deg_s,
    yaw_rate_deg_s,
    thrust_percent,
    rate=True,
)
```

It currently introduces no additional angular-rate sign inversion. Crazyflie
firmware retains responsibility for body-rate stabilization, motor mixing, and
motor actuation. The PC-side law does not directly command torque, angular
acceleration, or individual motors.

## 8. Stage 2 Equilibrium-Thrust Experiment

Stage 2 (`ValidationStage.EQUILIBRIUM_THRUST`) is an experiment-specific law in
`experiments/outer_loop_pid_validation.py`. It does **not** use the generic
`OuterLoopPIDController` to generate commands.

Initialization captures yaw `psi_0` and position for reference/logging.
Horizontal position, altitude, velocity, and body rates do not contribute to
the Stage 2 command. Direct attitude P feedback is:

```math
p_{cmd} = \operatorname{sat}_{L_{att}}
\left[s_{roll}K_{p,roll}(0-\phi)\right],
```

```math
q_{cmd} = \operatorname{sat}_{L_{att}}
\left[s_{pitch}K_{p,pitch}(0-\theta)\right],
```

```math
e_{\psi,2} = \operatorname{atan2}
\left(\sin(\psi_0-\psi),\cos(\psi_0-\psi)\right),
```

```math
r_{cmd} = \operatorname{sat}_{L_{yaw}}
\left[s_{yaw}K_{p,yaw}e_{\psi,2}\right].
```

Here `sat_L(v) = max(-L, min(v, L))`. Configurable polarities `s_roll`,
`s_pitch`, and `s_yaw` must be `-1` or `+1`. Roll and pitch share
`attitude_rate_limit_rad_s`; yaw uses `heading_rate_limit_rad_s`. Gains must be
positive, and both limits are configuration-capped at 10 deg/s. Configured
signs are not claimed as physically verified signs.

For elapsed time `t`, initial thrust `T_0`, equilibrium thrust `T_eq`, and ramp
duration `t_r`, the ramp before final bounds is

```math
T(t) =
\begin{cases}
T_0 + \dfrac{t}{t_r}(T_{eq}-T_0), & 0 \le t < t_r,\\
T_{eq}, & t \ge t_r.
\end{cases}
```

The transmitted value is

```math
T_{command}(t) = \operatorname{clamp}(T(t), T_{min}, T_{max}).
```

Validation requires `T_min <= T_0 < T_eq <= T_max`, positive ramp duration,
and nonnegative hold duration. Completion occurs at

```math
t \ge t_r + t_{hold}.
```

The program therefore ramps for `t_r`, holds `T_eq` for `t_hold`, and then lets
`UniversalRunner` execute its normal stop/disarm/shutdown sequence.

## 9. Validation Status

Automated tests cover representative world-to-body rotation, heading wrapping,
channel saturation, equilibrium feed-forward, configured sign application,
adapter conversion, estimator behavior, validation constraints, and Stage 2
attitude feedback, ramp, hold, and completion.

These tests establish software behavior, not aerodynamic performance. The
repository contains no durable evidence that the configured polarities,
equilibrium thrust, gains, stable hover, or trajectory tracking have been
successfully validated on hardware. The staged pipeline exists to perform that
work progressively; results should be recorded before physical behaviors are
claimed as validated.
