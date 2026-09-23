# Control Design

## Active Control Boundary

The active framework defines a backend-independent controller boundary:

```text
DroneState + Reference + dt
              |
              v
          Controller
              |
              v
   BodyRateThrustCommand
              |
              v
       CommandAdapter
              |
              v
   backend implementation
```

A controller maps the current state, desired reference, and update interval to
a `BodyRateThrustCommand`. No active controller implementation is currently
bundled with the framework; obsolete controller implementations are retained
under `legacy/`.

## State and Reference Conventions

`DroneState.position` and `DroneState.velocity` are world-frame quantities in
meters and meters/second. Attitude fields `roll`, `pitch`, and `yaw` are Euler
angles in radians.

State fields `p`, `q`, and `r` are body-frame angular rates about the body x,
y, and z axes in radians/second. They are not generally the Euler-angle
derivatives.

`Reference` contains desired position, velocity, yaw, yaw rate, and timestamp.
Individual controllers determine which reference fields they use.

## Body-Rate and Thrust Command

The active controller output is:

```text
BodyRateThrustCommand
    roll_rate
    pitch_rate
    yaw_rate
    thrust
```

Its semantics are:

```text
roll_rate   body x-axis angular-rate reference, rad/s
pitch_rate  body y-axis angular-rate reference, rad/s
yaw_rate    body z-axis angular-rate reference, rad/s
thrust      normalized collective thrust [0.0, 1.0]
```

These three rates are body-axis angular-rate references. They are not Euler
attitude angles and are not generally Euler-angle derivatives.

Collective thrust is a normalized command magnitude, not a world-frame
vertical-force command. Its hardware encoding belongs to the backend.

## Backend Responsibilities

Controllers produce commands in the framework units above. They do not apply
Crazyflie-specific units, signs, encodings, or API calls.

`CrazyflieCommandAdapter` converts each angular rate from radians/second to
degrees/second, encodes normalized thrust for the Crazyflie API, and calls:

```text
send_setpoint_manual(..., rate=True)
```

The Crazyflie firmware retains responsibility for body-rate stabilization and
motor mixing. The framework command does not directly represent torque,
angular acceleration, or individual motor output.

## Design Invariants

Active controllers and backends must preserve these rules:

* command field names identify their physical semantics explicitly;
* body-axis angular rates use radians/second at the controller boundary;
* normalized collective thrust remains within `[0.0, 1.0]`;
* hardware conversions remain inside command adapters;
* different physical command levels use distinct command types;
* active framework code does not depend on `legacy/`.
