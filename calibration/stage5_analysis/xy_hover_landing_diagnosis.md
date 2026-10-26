# First XY/Z hover landing diagnosis

Run: `20261026T080239_426849Z_cascaded-xy` (551 samples, 11.0001 s)

## Flight phases and tracking

| phase | samples | logged interval (s) | duration (s) | altitude range (m) | reference range (m) |
|---|---:|---:|---:|---:|---:|
| TAKEOFF | 100 | 0.0000–1.9801 | 1.9801 | 0.00009–0.20733 | 0.00006–0.19807 |
| HOVER | 150 | 2.0001–4.9802 | 2.9801 | 0.08316–0.19663 | 0.20006 |
| LANDING | 125 | 5.0002–7.4802 | 2.4800 | 0.11977–0.19124 | 0.20004–0.00165 |
| SETTLE | 25 | 7.5001–7.9801 | 0.4800 | 0.09251–0.11840 | 0.00006 |
| LANDING_CONFIRM | 151 | 8.0002–11.0001 | 2.9999 | 0.08257–0.12195 | 0.00006 |

During HOVER, vector XY tracking RMSE was 0.13216 m and altitude RMSE was
0.07561 m. Maximum horizontal displacement over the flight was 0.16734 m; final
displacement was 0.02689 m. Maximum altitude above its instantaneous reference
was 0.12189 m, occurring after the reference descended. No altitude, roll-rate,
pitch-rate, or yaw-rate command saturated.

## Landing diagnosis

Ground altitude was 0.000063 m and the touchdown-height threshold was 0.060063
m. After LANDING began, the minimum measured altitude was 0.082573 m. Thus the
vehicle never entered the touchdown-height band; the confirmation logic correctly
rejected an aircraft that remained airborne.

During LANDING_CONFIRM, altitude remained 0.08257–0.12195 m, vertical velocity
oscillated from -0.07214 to +0.06663 m/s, and thrust remained 0.41492–0.42407.
At termination it was at 0.11089 m and rising at 0.01325 m/s. With a ground-level
reference, the unchanged controller law approximately becomes
`u = 0.425 - 0.06*z - 0.06*vz`; near z=0.10 m and vz=0 it commands about 0.419.
The measured trajectory shows that this was sufficient to sustain a nonzero
ground-effect/feedforward equilibrium. The available downward thrust authority
was not saturated; it simply was not requested by the clamped reference.

## Minimal correction

Only after the nominal landing and settle profile completes, LANDING_CONFIRM now
continues a virtual altitude reference below the captured ground plane at the
existing landing rate, with the existing negative reference vertical velocity.
This progressively requests descent using the validated Kp/Kv law while keeping
the same thrust bounds and every hard safety check. Actual touchdown still
requires altitude <=0.06 m relative to ground and |vz| <=0.08 m/s continuously
for 0.5 s. Motor stop and disarm ordering is unchanged.

This is a targeted landing-sequencing correction, not evidence that the next
closed-loop landing will succeed. It does not change altitude or XY gains,
controller architecture, thrust limits, timeout, or touchdown thresholds.
