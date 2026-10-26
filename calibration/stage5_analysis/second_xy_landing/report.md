# Second XY/Z flight: altitude and landing diagnosis

Run: `20261026T080839_447421Z_cascaded-xy` (550 samples, 10.98 s).

![Altitude, vertical velocity, and thrust](altitude_velocity_thrust.svg)

## Secondary ascent

TAKEOFF ended near 0.2084 m with almost zero vertical velocity. During HOVER the
vehicle descended to 0.1840 m at 2.780 s. Smoothed vertical velocity crossed
positive at 2.820 s, exceeded +0.02 m/s at 2.920 s, and remained predominantly
positive. A stronger secondary ascent began around 4.18 s, when smoothed vz
exceeded +0.04 m/s. Altitude reached 0.2657 m at the final HOVER sample while
still rising at +0.0767 m/s.

At the 5.00 s HOVER-to-LANDING transition, thrust was already about 0.4160,
below the configured 0.425 equilibrium value, but upward velocity continued to
approximately +0.093 m/s. It reversed near 5.66 s, where altitude peaked at
0.3036 m against a 0.1471 m reference. The apparent delay from the landing
transition/thrust reduction to velocity reversal is about 0.66 s. Thrust had
been decreasing earlier, so this is a combined closed-loop vehicle/estimator
response delay, not a measured motor-only actuator delay.

## Equilibrium and damping interpretation

Near-zero-vz, low-acceleration HOVER samples cluster around 0.424–0.425 thrust,
so 0.425 remains credible as a local free-air feedforward estimate. It is not a
single invariant equilibrium near the floor, during changing battery voltage,
or during large vertical transients. The secondary ascent despite falling
thrust, followed by delayed reversal, is most consistent with an underdamped
vertical transient plus measurement/vehicle lag. The abrupt phase/reference
changes expose that response, but do not alone create upward thrust. One flight
cannot separate aerodynamic lag from estimator filtering precisely.

This evidence does not justify changing equilibrium thrust: reducing it would
also alter the already validated hover operating point. It suggests that Kv=0.06
does not fully damp the observed mode, but changing Kv is broader than necessary
to finish the current landing objective.

## Landing confirmation

The virtual below-ground reference operated correctly: it moved from the ground
reference at 8.00 s to -0.2385 m at 10.98 s, with reference vz=-0.08 m/s.
Commanded thrust consequently fell as low as 0.4044 without saturation.

The first descent reached 0.06960 m at 8.900 s, 9.66 mm above the 0.05994 m
touchdown threshold, then rebounded. A second descent reached about 0.0765 m at
10.74 s. At timeout, altitude was about 0.0805 m and commanded thrust was still
falling. The vehicle never entered the touchdown-height band, so confirmation
correctly failed; raising the threshold would risk declaring touchdown while
airborne.

Compared with the first XY/Z run, the second run had a larger secondary peak
(0.3036 versus 0.1912 m) but the new virtual reference produced materially more
downward authority (minimum confirmation thrust 0.4044 versus 0.4149) and a
lower minimum altitude (0.0696 versus 0.0826 m). Neither run saturated thrust.

## Smallest next correction

`LANDING_TIMEOUT_S` is extended from 3.0 to 4.0 seconds. Keep the below-ground
reference ramp, touchdown thresholds, controller gains, equilibrium thrust,
thrust bounds, XY loop, and all hard safety checks unchanged. The recommendation
is based on the aircraft reaching within 9.66 mm of the threshold and beginning
a second descent while substantial bounded downward authority remained.

This is preferable to relaxing touchdown detection or retuning the airborne
controller. It remains a diagnostic hypothesis for the next supervised run,
not proof that touchdown will occur within the additional second.
